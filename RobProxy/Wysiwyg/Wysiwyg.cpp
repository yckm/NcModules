#include "Wysiwyg.h"

static double r2d(double r) {
    return r * 180 / Pi;
}
/**
 * @创建人 dnp
 * @简介 获取两点之间的直线, 两点间距为一毫米
 * @参数 p1 起始点(tcp原点)
 * @参数 p2 终止点
 */
static std::vector<Eigen::Vector3d> getPointsBetween2Points(Eigen::Vector3d& p1, Eigen::Vector3d& p2) {
    p1 *= 1000;
    p2 *= 1000;

    Eigen::Vector3d v = p2 - p1;
    auto l = v.norm();
    v /= v.norm();
    
    auto times =(int)l;
    auto  remainder = l-times;

    std::vector<Eigen::Vector3d> targetPoints;
    for (int i = 0; i < 1 + times; i++) {
        if (i == times) {
            targetPoints.push_back((p1 + (i + remainder) * v) / 1000);
        }
        else {
            targetPoints.push_back((p1 + i * v) / 1000);
        }
    }
    return targetPoints;
}

/**
 * @创建人 dnp
 * @简介 根据tcp解算关节角度
 * @参数 inPose tcp位姿
 * @参数 inCurrjointsDeg 当前关节角度
 * @返回值 目标关节角度
 */
static std::tuple<bool, std::vector<double>> ik(std::vector<double> inPose, std::vector<double> inCurrjointsDeg) {
    std::vector<double> ip{ inPose[0] * 1000,inPose[1] * 1000,inPose[2] * 1000,r2d(inPose[3]),r2d(inPose[4]),r2d(inPose[5]) };
    std::vector<double> cjs{ r2d(inCurrjointsDeg[0]),r2d(inCurrjointsDeg[1]),r2d(inCurrjointsDeg[2]),r2d(inCurrjointsDeg[3]),r2d(inCurrjointsDeg[4]),r2d(inCurrjointsDeg[5]) };
    std::vector<double> joints;

    wysiwyg::Kinematrics::KinematicsWrapper kw;
    auto resp = kw.IKinematics(ip, cjs, joints);

    if (resp == 0) {
        std::tuple<bool, std::vector<double>> r = { false,joints };
        return r;        
    }

    for (int i = 0; i < joints.size(); i++) {
        joints[i]= joints[i] *Pi / 180;
    }
    std::tuple<bool, std::vector<double>> r = { true,joints };
    return r;
}



/**
* @创建人 dnp
* @简介 根据起始位置,计算两位置直线运动所需要的关节角度列表
* @参数 startPoseInBase (基座坐标系下)当前tcp位置(x,y,z,rx,ry,rz) 单位分别是m和弧度
* @参数 targetPoseInBase(基座坐标系下)目标tcp位置 (x,y,z,rx,ry,rz)
* @参数 curJoints 当前关节角度
* @返回值 运动到目标位置的一系列关节角度列表
*/
std::vector<std::vector<double>> wysiwyg::Wysiwyg::getMoveJoints(std::vector<double> startPoseInBase, std::vector<double> targetPoseInBase, std::vector<double> curJoints)
{
    Eigen::Vector3d p1, p2;
    p1 << startPoseInBase[0], startPoseInBase[1], startPoseInBase[2];
    p2 << targetPoseInBase[0], targetPoseInBase[1], targetPoseInBase[2];
    auto targetPoints = getPointsBetween2Points(p1, p2);


    // 计算每一个目标位姿的关节角度
    std::vector<std::vector<double>> targetJoints;
    std::vector<double> curJs = curJoints;
    int failCnt = 0,cnt=0;

    for (int i = 0; i < targetPoints.size(); i++) {
        if (cnt > 999) {
            break; // 轨迹池只能容纳1000个点,所以到达1000个点的时候就结束了
        }
        auto& p = targetPoints[i];
        std::vector<double> tcp = { p[0],p[1],p[2],startPoseInBase[3],startPoseInBase[4],startPoseInBase[5] };
        std::tuple<bool, std::vector<double>> j = ik(tcp, curJs);

        if (failCnt > 5) {
            LOG(INFO) << i << "逆计算关节角度失败5次,停止解算";
            return targetJoints;
        }

        if (!std::get<0>(j)) {
            failCnt++;
            LOG(INFO) << i << "逆计算关节角度失败";
            continue;
        }

        failCnt = 0;
        targetJoints.push_back(std::get<1>(j));
        cnt++;
    }
    return targetJoints;
}


std::vector<std::vector<double>> wysiwyg::Wysiwyg::calcJointss(std::vector<double>& armTcp, Eigen::Vector4d& v, std::vector<double>& curJoints, std::vector<double> range)
{
    // 基座坐标系下的起始位置
    Eigen::Matrix4d  matTcpInBase = tcpPose2mat(armTcp);
    Eigen::Matrix4d startPosInBase = matTcpInBase;
    Eigen::Vector4d targetPosInBase = startPosInBase * v; // 目标点在基座坐标系下的位置

#pragma region 判断是否超限
    double deltaX = startPosInBase(0, 3) - targetPosInBase(0, 3);
    double deltaY = startPosInBase(1, 3) - targetPosInBase(1, 3);
    double deltaZ = startPosInBase(2, 3) - targetPosInBase(2, 3);
    double dist = sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);
    if (dist > max(range[0], range[1])) {
        LOG(INFO) << "运动距离过大,停止.距离为: " << dist << "米";
        std::vector<std::vector<double>> fail;
        return fail;
    }
#pragma endregion

    std::vector<double> startPoseInBase = { matTcpInBase(0,3),matTcpInBase(1,3), matTcpInBase(2,3),armTcp[3],armTcp[4],armTcp[5] };
    std::vector<double> targetPoseInBase = { targetPosInBase(0),targetPosInBase(1), targetPosInBase(2),armTcp[3],armTcp[4],armTcp[5] };
    return getMoveJoints(startPoseInBase, targetPoseInBase, curJoints);
}

/**
* @创建人 dnp
* @简介 手眼同臂
* @参数 cam 观测相机
* @参数 xy 相机xy方向上的增量
* @参数 armTcp 默认tcp坐标(偏移角度皆为0的那个tcp)的坐标值
* @参数 curJoints 当前关节角度(单位弧度)
*/
wysiwyg::WysiwygData wysiwyg::Wysiwyg::onSameHand(Utils::Camera& cam,  std::vector<double> xy, std::vector<double> armTcp, std::vector<double> curJoints, std::vector<double> range)
{
    Eigen::Matrix3d m= cam.mat.block<3, 3>(0, 0);

    Eigen::Vector3d vc = { xy[0],xy[1],0 };
    auto vt = m * vc; 
    Eigen::Vector4d v = { vt[0],vt[1],vt[2],1 };

    wysiwyg::WysiwygData wd;
    wd.robId = cam.robId;
    wd.joints= calcJointss(armTcp, v, curJoints,range);
    return wd;
}


/**
* @创建人 dnp
* @简介 相机在主臂控制从臂运动
* @参数 cam 观测相机
* @参数 xy 相机xy方向上的增量
* @参数 moveArmJoints 运动臂关节角度
* @参数 observeArmTcp 观测臂默认tcp当前坐标
* @参数 moveArmTcp 运动臂默认tcp当前坐标
* @参数 camInMain 观测相机是否在主臂
*/
wysiwyg::WysiwygData wysiwyg::Wysiwyg::onDiffHand(Utils::Camera& cam, std::vector<double> xy, std::vector<double> observeArmTcp, std::vector<double> moveArmTcp, std::vector<double> moveArmJoints, std::vector<double> range)
{
    Eigen::Vector3d vc = { xy[0],xy[1],0};
    Eigen::Matrix4d moveTcp = tcpPose2mat(moveArmTcp);

    Eigen::Vector3d vInObserveBase =getR(tcpPose2mat(observeArmTcp))* getR(cam.mat) * vc;
    Eigen::Vector3d vInMoveBase =(cam.robId==1 ? getR(Utils::Config::getCamera("AssistMat2Main").mat) : getR(Utils::Config::getCamera("Main2AssistMat").mat)) *vInObserveBase;
    Eigen::Vector3d  vInMoveTcp0 = getR(moveTcp).transpose() * vInMoveBase;      
    Eigen::Vector4d vInMoveTcp = { vInMoveTcp0[0],vInMoveTcp0[1],vInMoveTcp0[2],1 };

    wysiwyg::WysiwygData wd;
    wd.robId = cam.robId==1?2:1;
    wd.joints= calcJointss(moveArmTcp, vInMoveTcp, moveArmJoints,range);
    return wd;
}

/**
* @创建人 dnp
* @简介 更加关节角度计算默认tcp位姿
* @参数 joints 关节角度(弧度)
* @返回值 tcp末端位姿
*/
std::vector<double> wysiwyg::Wysiwyg::getTcpFromJoints(std::vector<double>& joints)
{
    wysiwyg::Kinematrics::KinematicsWrapper kw;

    std::vector<double> jsDegree;
    for (int i = 0; i < 6; i++) {
        jsDegree.push_back(r2d(joints[i]));
    }

    std::vector<double> tcp;
    kw.FKinematics(jsDegree, tcp);

    tcp[0] /= 1000;
    tcp[1] /= 1000;
    tcp[2] /= 1000;
    tcp[3] = tcp[3] * Pi / 180;
    tcp[4] = tcp[4] * Pi / 180;
    tcp[5] = tcp[5] * Pi / 180;
    return tcp;
}
