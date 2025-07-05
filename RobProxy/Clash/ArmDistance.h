#pragma once
/*****************************************************************
 * @文件名称:ArmDistance.h
 * @创建人:dnp
 * @创建日期:2023-11-10
 * @简述:圆柱体距离计算
 *
 * @更改历史:
 * 日期: 2023年11月10日 作者: dnp 简述:
 *
*******************************************************************/

#define ClashCYLINDERDISTCALCAPI __declspec(dllexport)

#include <iostream>
#include <vector>
#include <memory>
#include <tuple>
#include <map>
#include <Eigen/Core>
#include <fcl/narrowphase/collision_object.h>
#include "Models.h"
#include "EnvDistance.h"

namespace Clash {
	class ClashCYLINDERDISTCALCAPI ArmDistance
	{
	private:		
		// 臂长
		std::vector<std::vector<double>> hs = {
			{ 75.0, 258.0 },
			{ 75.0, 187.0 + 75.0 },
			{ 75.0, 608.0 },
			{ 58.0, 150.0 + 58 },
			{ 58.0, 566.0 },
			{ 47.0, 126.0 + 47.0 },
			{ 47.0, 126.0 },
			{ 47.0, 113.0 }
		};

		std::vector<fcl::CollisionObject<double>*> arm1;
		std::vector<fcl::CollisionObject<double>*> arm2;
		EnvDistance env;
	public:
		/**
		 * @创建人:dnp
		 * @简述: 构造函数
		 * @参数:hs1 机械臂1的连杆
		 * @参数:hs2
		 * **/
		ArmDistance();
		~ArmDistance();

		/**
		 * @创建人:dnp
		 * @简述:设置关节角度
		 * @参数:armId 机器人Id(1,2)
		 * @参数:joints 关节角度列表
		 * **/
		void setArmJoints(int armId, std::vector<double> joints);

		/**
		 * @创建人:dnp
		 * @简述:设置关节角度
		 * @参数:armId 机器人Id(1,2)
		 * @参数:joints 关节角度列表
		 * @参数: baseTransMat 基座标变换矩阵
		 * **/
		void setArmJoints(int armId, std::vector<double> joints, Eigen::Matrix4d baseTransMat);

		/**
		 * @创建人:dnp
		 * @简述: 计算距离
		 * @返回值: 最小距离
		 * **/
		std::tuple<double, std::vector<double>> calcDistance();

		/**
		 * @创建人:dnp
		 * @简述: 计算环境模型和另一个碰撞链的距离
		 * @参数:arm 碰撞链
		 * @返回值: 最近距离
		 * **/
		void updateEnv(std::vector<Clash::CylinderModel>& cms);

		/**
		 * @创建人:dnp
		 * @简述: 计算机械臂 armId到环境的距离
		 * @参数:armId 机械臂ID
		 * @返回值: 机械臂到环境的距离
		 * **/
		std::tuple<double, std::vector<double>> toEnvDistance(int armId);

		/**
		 * @创建人 dnp
		 * @简介 获取双臂点云信息
		 */
		std::vector<fcl::CollisionObject<double>*>  getCollisionObjects();
	};
}
