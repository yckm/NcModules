#pragma once
#include <vector>
#include <chrono>
#include <thread>
#include "../../LidarHelper/LidarHelper.h"
#include "../../LidarHelper/Models.h"
#include "../Utils/Utils.h"
#include "ArmDistance.h"
#include "../Proxy.h"
using util = Utils::Utils;

namespace Clash {
	class DistanceDectector {
	private:
		// 点云服务器地址
		static std::string pcdServerUrl;
		static std::shared_ptr< Clash::ArmDistance>  distanceManager; // 距离计算器
		static int activatedArmId;
		static int minSecurityDistance; // 最小安全间距
		static Eigen::Matrix4d arm2TransMat;// 2号机械臂相对与1号机械臂的变换矩阵
	public:

		/**
		 * @创建人:dnp
		 * @简述:运行检测服务
		 * **/
		static void Run();

		/**
		 * @创建人:dnp
		 * @简述: 获取环境点云并更新DistanceCalcuator中环境的数据
		 * @参数: 安全相位间距(单位 mm)
		 * @返回值: 环境点云
		 * **/
		static BasicMsg upateEnv(int safeDist);

		/**
		 * @创建人:dnp
		 * @简述:设置特定机械臂为活动状态
		 * @参数:armId
		 * **/
		static void activateArm(int armId);

		/**
		 * @创建人:dnp
		 * @简述:设置机械臂的转换矩阵(因为多机械臂,需要将坐标化归为同一坐标系下,机械臂基座标到该坐标下有以转换矩阵)
		 * @参数:mat 转换矩阵
		 * **/
		static void setArmTransMat(Eigen::Matrix4d mat);
	};

#pragma region 实现
	Eigen::Matrix4d init() {
		Eigen::Matrix4d mat;
		mat << 1, 0, 0, 600,
			0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;
		return mat;
	}

#pragma region 变量
	std::shared_ptr< Clash::ArmDistance>  DistanceDectector::distanceManager = std::make_shared<Clash::ArmDistance>();
	std::string DistanceDectector::pcdServerUrl = "http://127.0.0.1:8000";
	int DistanceDectector::activatedArmId = 1;
	Eigen::Matrix4d DistanceDectector::arm2TransMat = init();
#pragma endregion

#pragma region 函数

	/**
	 * @创建人:dnp
	 * @简述: 获取环境点云并更新DistanceCalcuator中环境的数据
	 * @返回值: 环境点云
	 * **/
	BasicMsg DistanceDectector::upateEnv(int safeDist) {
		LidarHelper lh(pcdServerUrl);

		std::vector<Clash::CylinderModel> cls;
		for (auto& v : lh.getBoundryCylinder(safeDist)) {
			Clash::CylinderModel cld{ v.radius,v.height,v.trans };
			cls.push_back(cld);
		}

		distanceManager->updateEnv(cls);
		return BasicMsg{ "Success",1 };
	}

	/**
	 * @创建人:dnp
	 * @简述:设置特定机械臂为活动状态
	 * @参数:armId
	 * **/
	void DistanceDectector::activateArm(int armId) {
		if (armId == activatedArmId) {
			return;
		}
		activatedArmId = armId;
	}

	/**
	 * @创建人:dnp
	 * @简述:设置机械臂的转换矩阵(因为多机械臂,需要将坐标化归为同一坐标系下,机械臂基座标到该坐标下有以转换矩阵)
	 * @参数:armId 机械臂ID (取值为 1,2)
	 * @参数:mat 转换矩阵
	 * **/
	void DistanceDectector::setArmTransMat(Eigen::Matrix4d mat) {
		arm2TransMat = mat;
	}

	/**
	 * @创建人:dnp
	 * @简述:运行检测服务
	 * **/
	void DistanceDectector::Run() {
		const int sleepTime = 10;// 每个10ms计算一次

		long st = util::GetTsMs();
		long costMm = 0; // 耗时

		std::vector<float> js1Old; // 保存上一次更新时候用的关节
		std::vector<float> js2Old;

		while (true)
		{
			st = util::GetTsMs();
			if (costMm < sleepTime) {
				std::this_thread::sleep_for(std::chrono::milliseconds(sleepTime - costMm));
			}

			// 获取双臂角度
			auto js = mainthread::Proxy::GetJoints();
			auto& js1 = js[0];
			auto& js2 = js[1];

			// 更新机械臂关节角度
			bool updated = false; // 如果没有更新就不进行计算了
			if (!js1.empty() && js1 != js1Old) {
				js1Old = js1;
				distanceManager->setArmJoints(1, Utils::Utils::toArrd(js1));
				updated = true;
			}
			if (!js2.empty() && js2 != js2Old) {
				js2Old = js2;
				distanceManager->setArmJoints(2, Utils::Utils::toArrd(js2), arm2TransMat);
				updated = true;
			}

			if (!updated) {
				costMm = 0;
				continue;
			}

			// 有一个没有机器人关节角度的话,根本没法算出是否要相撞
			auto distArm2Arm = distanceManager->calcDistance();// 机械臂到机械臂的距离
			auto distArm2Env = distanceManager->toEnvDistance(activatedArmId); // 运动机械臂到电子围栏的距离

			// 设置主线程碰撞距离
			mainthread::Proxy::UpdateClashDistance(std::get<0>(distArm2Arm), std::get<0>(distArm2Env));

			costMm = util::GetTsMs() - st;
		}
	}
#pragma endregion
#pragma endregion
}
