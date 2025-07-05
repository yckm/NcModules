#include "ArmDistance.h"
#include <string>
#include <sys/types.h>
#include <stdio.h>
#include <errno.h>
#include <cmath>
#include <fcl/narrowphase/distance.h>
#include <fcl/narrowphase/distance_request.h>
#include <fcl/narrowphase/distance_result.h>
#include <fcl/math/constants.h>
#include "DistanceHelper.h"
#include "Arm.h"

namespace Clash {
	ArmDistance::ArmDistance()
	{
		// 构造机械臂连杆
		for (const auto& rh : hs) {
			std::shared_ptr<fcl::CollisionGeometry<double>> g1 = std::make_shared<fcl::Cylinder<double>>(rh[0], rh[1]);
			std::shared_ptr<fcl::CollisionGeometry<double>> g2 = std::make_shared<fcl::Cylinder<double>>(rh[0], rh[1]);

			arm1.push_back(new fcl::CollisionObject<double>(g1));
			arm2.push_back(new fcl::CollisionObject<double>(g2));
		}
	}

	ArmDistance::~ArmDistance()
	{
		if (arm1.size() > 0) {
			int len = arm1.size();
			for (int i = 0; i < len; i++) {
				delete arm1[i];
				delete arm2[i];
			}
		}
	}

	/**
	 * @创建人 dnp
	 * @简介 从4x4矩阵中获取旋转矩阵
	 * @参数 mat
	 * @返回值 
	 */
	Eigen::Matrix3d getR(Eigen::Matrix4d mat) {
		Eigen::Matrix3d m;
		m << mat(0, 0), mat(0, 1), mat(0, 2),
			mat(1, 0), mat(1, 1), mat(1, 2),
			mat(2, 0), mat(2, 1), mat(2, 2);
		return m;
	}

	/**
	 * @创建人 dnp
	 * @简介 设置机械臂的姿态
	 */
	void setArmPose(fcl::CollisionObject<double>* arm, Eigen::Matrix4d& start, Eigen::Matrix4d& end) {
		double x = 0.5 * end(0,3) + 0.5 * start(0,3);
		double y = 0.5 * end(1,3) + 0.5 * start(1,3);
		double z = 0.5 * end(2, 3) + 0.5 * start(2, 3);
		auto r = getR(end);

		fcl::Vector3d trans(x, y, z);
		/*std::cout << "------------------------------------" << std::endl;
		std::cout << x <<  "  " << y << "  " << z << std::endl;
		std::cout << r << std::endl;*/

		arm->setTranslation(trans);
		arm->setRotation(r);
	
	}

	/**
	 * @创建人 dnp
	 * @简介 设置机械臂的姿态
	 */
	void setArmPose(fcl::CollisionObject<double>* arm, Eigen::Matrix4d& start, Eigen::Matrix4d& end,Eigen::Matrix3d R) {
		double x = 0.5 * end(0, 3) + 0.5 * start(0, 3);
		double y = 0.5 * end(1, 3) + 0.5 * start(1, 3);
		double z = 0.5 * end(2, 3) + 0.5 * start(2, 3);
		auto r = getR(start)*R;

		fcl::Vector3d trans(x, y, z);
		/*std::cout << "------------------------------------" << std::endl;
		std::cout << x << "  " << y << "  " << z << std::endl;
		std::cout << r << std::endl;*/

		arm->setTranslation(trans);
		arm->setRotation(r);

	}


	void ArmDistance::setArmJoints(int armId, std::vector<double> joints)
	{
		std::vector<fcl::CollisionObject<double>*>& arm = armId == 1 ? arm1 : arm2;

#pragma region 获取每一节点的坐标
		std::vector<double> armLengths;
		for (const auto& v : hs) {
			armLengths.push_back(v[1]);
		}
		Arm am(armLengths);
		std::vector<Eigen::Matrix4d> ms = am.GetArmPose(joints);
#pragma endregion

		setArmPose(arm[0], ms[0], ms[1]);
		setArmPose(arm[1], ms[1], ms[2], Arm::rz3(joints[0])*Arm::rx3(Pi /2));
		setArmPose(arm[2], ms[2], ms[3]);
		setArmPose(arm[3], ms[3], ms[4], -Arm::rx3(Pi / 2));
		setArmPose(arm[4], ms[4], ms[5]);
		setArmPose(arm[5], ms[5], ms[6], Arm::rx3(Pi / 2));
		setArmPose(arm[6], ms[6], ms[7]);
		setArmPose(arm[7], ms[7], ms[8], Arm::rz3(joints[4])*Arm::rx3(Pi / 2));

		// Clash::DistanceHelper::SampleFclModel(arm, 999, "C:\\dxq\\fcljsons\\");	
	}
	void ArmDistance::setArmJoints(int armId, std::vector<double> joints, Eigen::Matrix4d baseTransMat)
	{
		setArmJoints(armId, joints);

		// 将整个机械臂进行变换(等价与将机械臂的每个连杆进行相同的变换)
		std::vector<fcl::CollisionObject<double>*>& arm = armId == 1 ? arm1 : arm2;
		fcl::Matrix3d rat;
		rat << baseTransMat(0, 0), baseTransMat(0, 1), baseTransMat(0, 2),
			baseTransMat(1, 0), baseTransMat(1, 1), baseTransMat(1, 2),
			baseTransMat(2, 0), baseTransMat(2, 1), baseTransMat(2, 2);

		float dx, dy, dz;
		dx = baseTransMat(0, 3);
		dy = baseTransMat(1, 3);
		dz = baseTransMat(2, 3);

		for (int i = 0; i < arm.size(); i++) {
			auto mt = baseTransMat * arm[i]->getTransform().matrix();
			fcl::Matrix3d rotation;
			rotation << mt(0, 0), mt(0, 1), mt(0, 2),
				mt(1, 0), mt(1, 1), mt(1, 2),
				mt(2, 0), mt(2, 1), mt(2, 2);
			fcl::Vector3d mv(mt(0, 3), mt(1, 3), mt(2, 3));
			arm[i]->setRotation(rotation);
			arm[i]->setTranslation(mv);
		}
	}

	std::tuple<double, std::vector<double>> ArmDistance::calcDistance()
	{
		return DistanceHelper::calcDistance(arm1, arm2);
	}

	void ArmDistance::updateEnv(std::vector<Clash::CylinderModel>& cms)
	{
		env.update(cms);
	}

	std::tuple<double, std::vector<double>> ArmDistance::toEnvDistance(int armId)
	{
		if (armId == 1) {
			return env.calcDistance(arm1);
		}
		return env.calcDistance(arm2);
	}

	std::vector<fcl::CollisionObject<double>*>  ArmDistance::getCollisionObjects()
	{
		std::vector<fcl::CollisionObject<double>*> arms;
		for (const auto& ptr : arm1) {
			arms.push_back(ptr);
		}
		for (const auto& ptr : arm2) {
			arms.push_back(ptr);
		}
		for (const auto& ptr : env.env) {
			arms.push_back(ptr);
		}
		return arms;
	}
}