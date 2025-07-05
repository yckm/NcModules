#pragma once
/*****************************************************************
 * @文件名称:EnvDistance.h
 * @创建人:dnp
 * @创建日期:2023-11-14
 * @简述: 安全线间距
 *
 * @更改历史:
 * 日期: 作者: 简述:
 *
*******************************************************************/

#include <iostream>
#include <vector>
#include <memory>
#include <tuple>
#include <map>
#include <Eigen/Core>
#include <fcl/narrowphase/collision_object.h>
#include "Models.h"
#define ClashENVDISTCALCAPI __declspec(dllexport)
namespace Clash {
	class ClashENVDISTCALCAPI EnvDistance
	{
	public:
		std::vector<fcl::CollisionObject<double>*> env;

		/**
		 * @创建人:dnp
		 * @简述:更新环境模型(创建碰撞链)
		 * @参数:cms 原件圆柱模型
		 * **/
		void update(std::vector <Clash::CylinderModel>& cms);

		/**
		 * @创建人:dnp
		 * @简述: 计算环境模型和另一个碰撞链的距离
		 * @参数:arm 碰撞链
		 * @返回值: 最近距离
		 * **/
		std::tuple<double, std::vector<double>> calcDistance(std::vector<fcl::CollisionObject<double>*>& chains);
	};
}
