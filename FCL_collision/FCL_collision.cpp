#include "include/test_fcl_utility.h"
#include "Eigen/Core"
#include <iostream>
#include <fstream>
//#include <sstream>
#include <vector>
//#include <Windows.h> 
//#include "config.h"
#include <tuple>
#include "fcl/broadphase/broadphase_dynamic_AABB_tree.h"
#include "fcl/broadphase/broadphase_dynamic_AABB_tree_array.h"
#include "fcl/broadphase/default_broadphase_callbacks.h"
#include "include/DistanceHelper.h"
#include "fcl/narrowphase/detail/traversal/collision_node.h"

LARGE_INTEGER cpuFreq;
LARGE_INTEGER startTime;
LARGE_INTEGER endTime;
typedef fcl::BVHModel<fcl::OBBRSSd> Model;


uint32_t change_arm(int side, std::vector<double> tr)
{

}

double toRad(double degree)
{
    return degree * 3.14159265358979323846 / 180.0;
}

std::vector<fcl::CollisionObjectd*> loadArm()
{
    std::vector<fcl::CollisionObjectd*> arm;
    std::vector<fcl::Vector3d> pb_l, p1_l, p2_l, p3_l, p4_l, p5_l, p6_l;
    std::vector<fcl::Triangle> tb_l, t1_l, t2_l, t3_l, t4_l, t5_l, t6_l;
    fcl::test::loadOBJFile("D:\\DDcode\\110kv-220kv-3d\\3DModel\\mesh\\baseL.obj", pb_l, tb_l);
    fcl::test::loadOBJFile("D:\\DDcode\\110kv-220kv-3d\\3DModel\\mesh\\j1L.obj", p1_l, t1_l);
    fcl::test::loadOBJFile("D:\\DDcode\\110kv-220kv-3d\\3DModel\\mesh\\j2L.obj", p2_l, t2_l);
    fcl::test::loadOBJFile("D:\\DDcode\\110kv-220kv-3d\\3DModel\\mesh\\j3L.obj", p3_l, t3_l);
    fcl::test::loadOBJFile("D:\\DDcode\\110kv-220kv-3d\\3DModel\\mesh\\j4L.obj", p4_l, t4_l);
    fcl::test::loadOBJFile("D:\\DDcode\\110kv-220kv-3d\\3DModel\\mesh\\j5L.obj", p5_l, t5_l);
    fcl::test::loadOBJFile("D:\\DDcode\\110kv-220kv-3d\\3DModel\\mesh\\j6L.obj", p6_l, t6_l);

    std::vector<fcl::Vector3d> pb_r, p1_r, p2_r, p3_r, p4_r, p5_r, p6_r;
    std::vector<fcl::Triangle> tb_r, t1_r, t2_r, t3_r, t4_r, t5_r, t6_r;
    fcl::test::loadOBJFile("D:\\DDcode\\110kv-220kv-3d\\3DModel\\mesh\\baseR.obj", pb_r, tb_r);
    fcl::test::loadOBJFile("D:\\DDcode\\110kv-220kv-3d\\3DModel\\mesh\\j1R.obj", p1_r, t1_r);
    fcl::test::loadOBJFile("D:\\DDcode\\110kv-220kv-3d\\3DModel\\mesh\\j2R.obj", p2_r, t2_r);
    fcl::test::loadOBJFile("D:\\DDcode\\110kv-220kv-3d\\3DModel\\mesh\\j3R.obj", p3_r, t3_r);
    fcl::test::loadOBJFile("D:\\DDcode\\110kv-220kv-3d\\3DModel\\mesh\\j4R.obj", p4_r, t4_r);
    fcl::test::loadOBJFile("D:\\DDcode\\110kv-220kv-3d\\3DModel\\mesh\\j5R.obj", p5_r, t5_r);
    fcl::test::loadOBJFile("D:\\DDcode\\110kv-220kv-3d\\3DModel\\mesh\\j6R.obj", p6_r, t6_r);

    
    std::shared_ptr<Model> geomb_l = std::make_shared<Model>();
    geomb_l->beginModel();
    geomb_l->addSubModel(pb_l, tb_l);
    geomb_l->endModel();
    std::shared_ptr<Model> geom1_l = std::make_shared<Model>();
    geom1_l->beginModel();
    geom1_l->addSubModel(p1_l, t1_l);
    geom1_l->endModel();
    std::shared_ptr<Model> geom2_l = std::make_shared<Model>();
    geom2_l->beginModel();
    geom2_l->addSubModel(p2_l, t2_l);
    geom2_l->endModel();
    std::shared_ptr<Model> geom3_l = std::make_shared<Model>();
    geom3_l->beginModel();
    geom3_l->addSubModel(p3_l, t3_l);
    geom3_l->endModel();
    std::shared_ptr<Model> geom4_l = std::make_shared<Model>();
    geom4_l->beginModel();
    geom4_l->addSubModel(p4_l, t4_l);
    geom4_l->endModel();
    std::shared_ptr<Model> geom5_l = std::make_shared<Model>();
    geom5_l->beginModel();
    geom5_l->addSubModel(p5_l, t5_l);
    geom5_l->endModel();
    std::shared_ptr<Model> geom6_l = std::make_shared<Model>();
    geom6_l->beginModel();
    geom6_l->addSubModel(p6_l, t6_l);
    geom6_l->endModel();
    fcl::CollisionObjectd* base_l = new fcl::CollisionObjectd(geomb_l);
    arm.push_back(base_l);
    fcl::CollisionObjectd* j1_l = new fcl::CollisionObjectd(geom1_l);
    arm.push_back(j1_l);
    fcl::CollisionObjectd* j2_l = new fcl::CollisionObjectd(geom2_l);
    arm.push_back(j2_l);
    fcl::CollisionObjectd* j3_l = new fcl::CollisionObjectd(geom3_l);
    arm.push_back(j3_l);
    fcl::CollisionObjectd* j4_l = new fcl::CollisionObjectd(geom4_l);
    arm.push_back(j4_l);
    fcl::CollisionObjectd* j5_l = new fcl::CollisionObjectd(geom5_l);
    arm.push_back(j5_l);
    fcl::CollisionObjectd* j6_l = new fcl::CollisionObjectd(geom6_l);
    arm.push_back(j6_l);

    std::shared_ptr<Model> geomb_r = std::make_shared<Model>();
    geomb_r->beginModel();
    geomb_r->addSubModel(pb_r, tb_r);
    geomb_r->endModel();
    std::shared_ptr<Model> geom1_r = std::make_shared<Model>();
    geom1_r->beginModel();
    geom1_r->addSubModel(p1_r, t1_r);
    geom1_r->endModel();
    std::shared_ptr<Model> geom2_r = std::make_shared<Model>();
    geom2_r->beginModel();
    geom2_r->addSubModel(p2_r, t2_r);
    geom2_r->endModel();
    std::shared_ptr<Model> geom3_r = std::make_shared<Model>();
    geom3_r->beginModel();
    geom3_r->addSubModel(p3_r, t3_r);
    geom3_r->endModel();
    std::shared_ptr<Model> geom4_r = std::make_shared<Model>();
    geom4_r->beginModel();
    geom4_r->addSubModel(p4_r, t4_r);
    geom4_r->endModel();
    std::shared_ptr<Model> geom5_r = std::make_shared<Model>();
    geom5_r->beginModel();
    geom5_r->addSubModel(p5_r, t5_r);
    geom5_r->endModel();
    std::shared_ptr<Model> geom6_r = std::make_shared<Model>();
    geom6_r->beginModel();
    geom6_r->addSubModel(p6_r, t6_r);
    geom6_r->endModel();
    fcl::CollisionObjectd* base_r = new fcl::CollisionObjectd(geomb_r);
    arm.push_back(base_r);
    fcl::CollisionObjectd* j1_r = new fcl::CollisionObjectd(geom1_r);
    arm.push_back(j1_r);
    fcl::CollisionObjectd* j2_r = new fcl::CollisionObjectd(geom2_r);
    arm.push_back(j2_r);
    fcl::CollisionObjectd* j3_r = new fcl::CollisionObjectd(geom3_r);
    arm.push_back(j3_r);
    fcl::CollisionObjectd* j4_r = new fcl::CollisionObjectd(geom4_r);
    arm.push_back(j4_r);
    fcl::CollisionObjectd* j5_r = new fcl::CollisionObjectd(geom5_r);
    arm.push_back(j5_r);
    fcl::CollisionObjectd* j6_r = new fcl::CollisionObjectd(geom6_r);
    arm.push_back(j6_r);
    
    return arm;
}


void set_transform(fcl::CollisionObjectd* obj, double x, double y, double z, double rx, double ry, double rz)
{
    fcl::Matrix3d R;
    fcl::Vector3d T(x, y, z);
    fcl::test::eulerToMatrix(toRad(rz), toRad(ry), toRad(rx), R);
    obj->setTransform(R, T);
}

Eigen::Matrix4d matMv(double x, double y, double z)
{
    Eigen::Matrix4d mat;
    mat << 1, 0, 0, x,
        0, 1, 0, y,
        0, 0, 1, z,
        0, 0, 0, 1;
    return  mat;
}

Eigen::Matrix4d rz(double theta)
{
    Eigen::Matrix4d mat;
    mat << std::cos(toRad(theta)), -std::sin(toRad(theta)), 0, 0,
        std::sin(toRad(theta)), std::cos(toRad(theta)), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    return  mat;
}

Eigen::Matrix4d ry(double theta)
{
    Eigen::Matrix4d mat;
    mat << std::cos(toRad(theta)), 0, std::sin(toRad(theta)), 0,
        0, 1, 0, 0,
        -std::sin(toRad(theta)), 0, std::cos(toRad(theta)), 0,
        0, 0, 0, 1;
    return  mat;
}

Eigen::Matrix4d rx(double theta)
{
    Eigen::Matrix4d mat;
    mat << 1, 0, 0, 0,
        0, std::cos(toRad(theta)), -std::sin(toRad(theta)), 0,
        0, std::sin(toRad(theta)), std::cos(toRad(theta)), 0,
        0, 0, 0, 1;
    return  mat;
}

std::vector<Eigen::Matrix4d> GetArmPose(std::vector<double> joints, int armId)
{
    double j1, j2, j3, j4, j5, j6;
    j1 = joints[0];
    j2 = joints[1];
    j3 = joints[2];
    j4 = joints[3];
    j5 = joints[4];
    j6 = joints[5];

    std::vector<double> dx, dy, dz;
    dx = { 390,390,388.31,389.16,388.84,389.15 };
    dy = { 100,113,206.76,138.83,260.58,258.76 };
    dz = { 0,159.05,773.53,1337.85,1336.04,1464.14 };

    if (armId == 1)  //left
    {
        for (int i = 0; i < 6; i++)
        {
            dx[i] = -dx[i];
            dy[i] = -dy[i];
        }
        j2 = -j2;
        j3 = -j3;
        j4 = -j4;
        j6 = -j6;
    }

    Eigen::Matrix4d m1 = matMv(dx[0], dy[0], dz[0]) * rz(j1) * matMv(-dx[0], -dy[0], -dz[0]);
    Eigen::Matrix4d m2 = m1 * matMv(dx[1], dy[1], dz[1]) * ry(j2) * matMv(-dx[1], -dy[1], -dz[1]);
    Eigen::Matrix4d m3 = m2 * matMv(dx[2], dy[2], dz[2]) * ry(j3) * matMv(-dx[2], -dy[2], -dz[2]);
    Eigen::Matrix4d m4 = m3 * matMv(dx[3], dy[3], dz[3]) * ry(j4) * matMv(-dx[3], -dy[3], -dz[3]);
    Eigen::Matrix4d m5 = m4 * matMv(dx[4], dy[4], dz[4]) * rz(j5) * matMv(-dx[4], -dy[4], -dz[4]);
    Eigen::Matrix4d m6 = m5 * matMv(dx[5], dy[5], dz[5]) * ry(j6) * matMv(-dx[5], -dy[5], -dz[5]);
    std::vector < Eigen::Matrix4d> mats = { m1, m2, m3, m4, m5, m6 };
    return mats;
}

void setArmPose(fcl::CollisionObject<double>* arm, Eigen::Matrix4d& mt)
{
    double x = mt(0, 3);
    double y = mt(1, 3);
    double z = mt(2, 3);
    fcl::Vector3d trans(x, y, z);
    fcl::Matrix3d m;
    m << mt(0, 0), mt(0, 1), mt(0, 2),
        mt(1, 0), mt(1, 1), mt(1, 2),
        mt(2, 0), mt(2, 1), mt(2, 2);

    arm->setTransform(m, trans);
}

void main()
{
    std::vector<fcl::CollisionObjectd*> a;
    a = loadArm();

    fcl::BroadPhaseCollisionManagerd* manager1 = new fcl::DynamicAABBTreeCollisionManagerd();
    fcl::BroadPhaseCollisionManagerd* manager2 = new fcl::DynamicAABBTreeCollisionManagerd();
    for (int i = 0; i < a.size()/2; i++)
    {
        a[i]->computeAABB();
        a[i + 7]->computeAABB();
        //manager1->registerObject(a[i]);
        manager2->registerObject(a[i+7]);
    }
    
    std::vector< fcl::DefaultDistanceData<double>> distance_data;
    for (int i = 0; i < 7; i++)
    {
        fcl::DefaultDistanceData<double> distance;
        distance_data.push_back(distance);
    }
    //fcl::DistanceRequestd request;
    //fcl::DistanceResultd result;
    //fcl::distance(a[1], a[8], request, result);
    //std::cout << "min_distance:" << result.min_distance << std::endl;
    /*QueryPerformanceFrequency(&cpuFreq);
    QueryPerformanceCounter(&startTime);*/
    //距离检测
    
    
    
    //fcl::Matrix3d rotationMatrix;
    //rotationMatrix << -1, 0, 0, 0, -1, 0, 0, 0, 1;
    //a[1]->setRotation(rotationMatrix);
    //manager1->update();

    /*set_transform(a[0], -390, -100, 0, 0, 0, 180);
    set_transform(a[1], -390, -100, 0, 0, 0, 180);
    set_transform(a[2], -390, -100, 0, 0, 0, 180);
    set_transform(a[3], -390, -100, 0, 0, 0, 180);
    set_transform(a[4], -390, -100, 0, 0, 0, 180);
    set_transform(a[5], -390, -100, 0, 0, 0, 180);
    set_transform(a[6], -390, -100, 0, 0, 0, 180);*/

    /*Eigen::Matrix4d m0bl;
    m0bl << 1, 0, 0, -390,
        0, 1, 0, -100,
        0, 0, 1, 0,
        0, 0, 0, 1;
    Eigen::Matrix4d m1l;
    m1l << std::cos(toRad(90)), -std::sin(toRad(90)), 0, 0,
        std::sin(toRad(90)), std::cos(toRad(90)), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    Eigen::Matrix4d m1l_1;
    m1l_1 << 1, 0, 0, 390,
        0, 1, 0, 100,
        0, 0, 1, 0,
        0, 0, 0, 1;

    Eigen::Matrix4d m1 = m0bl * m1l * m1l_1;
    fcl::Matrix3d m;
    m << m1(0, 0), m1(0, 1), m1(0, 2),
        m1(1, 0), m1(1, 1), m1(1, 2),
        m1(2, 0), m1(2, 1), m1(2, 2);
    fcl::Vector3d t(m1(0,3), m1(1, 3), m1(2, 3));*/

    std::vector<double> joint = { 90,50,-80,-90,90,0 };
    std::vector<Eigen::Matrix4d> ms1 = GetArmPose(joint,1);

    for (int i = 0; i < 6; i++)
    {
        setArmPose(a[i + 1], ms1[i]);
    }

    joint = { 90,45,-45,-90,90,0 };
    std::vector<Eigen::Matrix4d> ms2 = GetArmPose(joint, 2);

    for (int i = 0; i < 6; i++)
    {
        setArmPose(a[i + 8], ms2[i]);
    }

    QueryPerformanceFrequency(&cpuFreq);
    QueryPerformanceCounter(&startTime);
    //距离检测
    for (int i = 0; i < 7; i++)
    {
        manager2->distance(a[i], &distance_data[i], fcl::DefaultDistanceFunction);
    }
    QueryPerformanceCounter(&endTime);
    double last = (((endTime.QuadPart - startTime.QuadPart) * 1000000) / cpuFreq.QuadPart);
    std::cout << "min_distance: ";
    for (int i = 0; i < 7; i++)
    {
        std::cout << distance_data[i].result.min_distance << " ";
    }
    std::cout << "\nTotal time:" << last << "us, " << std::endl;

    /*for (int i = 0; i < 7; i++)
    {
        fcl::distance(a[i], a[i+7],distance_data[i].request,distance_data[i].result);
    }
    QueryPerformanceCounter(&endTime);
    double last = (((endTime.QuadPart - startTime.QuadPart) * 1000000) / cpuFreq.QuadPart);
    std::cout << "min_distance: ";
    for (int i = 0; i < 7; i++)
    {
        std::cout << distance_data[i].result.min_distance << " ";
    }
    std::cout << "\nTotal time:" << last << "us, " << std::endl;*/

    // 臂自身距离检测
    fcl::BroadPhaseCollisionManagerd* s_manager_to6_l = new fcl::DynamicAABBTreeCollisionManagerd();
    s_manager_to6_l->registerObject(a[0]);
    s_manager_to6_l->registerObject(a[1]);
    s_manager_to6_l->registerObject(a[2]);
    s_manager_to6_l->registerObject(a[3]);

    fcl::BroadPhaseCollisionManagerd* s_manager_to5_l = new fcl::DynamicAABBTreeCollisionManagerd();
    s_manager_to5_l->registerObject(a[1]);
    s_manager_to5_l->registerObject(a[2]);

    fcl::BroadPhaseCollisionManagerd* s_manager_to3_l = new fcl::DynamicAABBTreeCollisionManagerd();
    s_manager_to3_l->registerObject(a[0]);
    s_manager_to3_l->registerObject(a[1]);

    fcl::BroadPhaseCollisionManagerd* s_manager_to6_r = new fcl::DynamicAABBTreeCollisionManagerd();
    s_manager_to6_r->registerObject(a[7]);
    s_manager_to6_r->registerObject(a[8]);
    s_manager_to6_r->registerObject(a[9]);
    s_manager_to6_r->registerObject(a[10]);

    fcl::BroadPhaseCollisionManagerd* s_manager_to5_r = new fcl::DynamicAABBTreeCollisionManagerd();
    s_manager_to5_r->registerObject(a[8]);
    s_manager_to5_r->registerObject(a[9]);

    fcl::BroadPhaseCollisionManagerd* s_manager_to3_r = new fcl::DynamicAABBTreeCollisionManagerd();
    s_manager_to3_r->registerObject(a[7]);
    s_manager_to3_r->registerObject(a[8]);

    fcl::DefaultDistanceData<double> distance;
    distance_data.push_back(distance);
    QueryPerformanceFrequency(&cpuFreq);
    QueryPerformanceCounter(&startTime);

    s_manager_to6_l->distance(a[6], &distance_data[0], fcl::DefaultDistanceFunction);
    s_manager_to5_l->distance(a[5], &distance_data[1], fcl::DefaultDistanceFunction);
    fcl::distance(a[4], a[2], distance_data[2].request, distance_data[2].result);
    s_manager_to3_l->distance(a[3], &distance_data[3], fcl::DefaultDistanceFunction);

    s_manager_to6_r->distance(a[13], &distance_data[4], fcl::DefaultDistanceFunction);
    s_manager_to5_r->distance(a[12], &distance_data[5], fcl::DefaultDistanceFunction);
    fcl::distance(a[11], a[9], distance_data[6].request, distance_data[6].result);
    s_manager_to3_r->distance(a[10], &distance_data[7], fcl::DefaultDistanceFunction);
    QueryPerformanceCounter(&endTime);
    last = (((endTime.QuadPart - startTime.QuadPart) * 1000000) / cpuFreq.QuadPart);
    std::cout << "self min distance: ";
    for (int i = 0; i < 8; i++)
    {
        std::cout << distance_data[i].result.min_distance << " ";
    }
    std::cout << "\nTotal time:" << last << "us, " << std::endl;


    //std::vector< fcl::DefaultDistanceData<double>> distance_data1;
    //for (int i = 0; i < 7; i++)
    //{
    //    fcl::DefaultDistanceData<double> distance;
    //    distance_data.push_back(distance);
    //}

    //QueryPerformanceFrequency(&cpuFreq);
    //QueryPerformanceCounter(&startTime);
    ////距离检测
    //for (int i = 0; i < 7; i++)
    //{
    //    fcl::distance(a[i], a[i + 7], distance_data1[i].request, distance_data1[i].result);
    //}
    //QueryPerformanceCounter(&endTime);
    //last = (((endTime.QuadPart - startTime.QuadPart) * 1000000) / cpuFreq.QuadPart);
    //std::cout << "min_distance: ";
    //for (int i = 0; i < 7; i++)
    //{
    //    std::cout << distance_data1[i].result.min_distance << " ";
    //}
    //std::cout << "\nTotal time:" << last << "us, " << std::endl;

    auto jsonArr = Clash::DistanceHelper::sampleFclModelToJson1(a, 59999);
    Clash::DistanceHelper::saveSampleFclModel(jsonArr, "D:\\DDcode\\110kv-220kv-3d\\3DModel\\json\\file\\");
}
