#pragma once
/*****************************************************************
 * @文件名称:DistanceHelper.h
 * @创建人:dnp
 * @创建日期:2023-11-14
 * @简述:距离助手
 *
 * @更改历史:
 * 日期: 作者: 简述:
 *
*******************************************************************/
#include <vector>
#include <fcl/narrowphase/distance.h>
#include <fcl/narrowphase/distance_request.h>
#include <fcl/narrowphase/distance_result.h>
#include <fcl/math/constants.h>
//#include "Coord.h"
namespace Clash {
	class DistanceHelper
	{
	public:
		/**
		 * @创建人:dnp
		 * @简述:计算两组碰撞链之间的距离
		 * @参数:chain1 碰撞链条1
		 * @参数:chain2 碰撞链条2
		 * @返回值: 两链条之间最近的最近(碰撞了距离为-1, 无效检测距离为-2)
		 * **/
		static std::tuple<double, std::vector<double>> calcDistance(std::vector<fcl::CollisionObject<double>*>& chain1, std::vector<fcl::CollisionObject<double>*>& chain2);

		/**
		 * @创建人 dnp
		 * @简介 将距离结果中最近的两个点提前出来
		 * @参数 nps 安放结果的数组
		 * @参数 result 距离结果
		 */
		static void extract_nearest_points(std::vector<double>& nps, fcl::DistanceResultd& result);

		/**
		 * 保存采样的json文件.
		 */
		static void saveSampleFclModel(std::vector<std::string>& arr, std::string savePath);

		/**
		 * 保存采样的json文件.
		 */
		static void saveSampleFclModel(std::vector<std::string>& arr, std::string savePath,std::vector<double> nearest_points);

		/**
		 * 对fc碰撞模型进行采样.
		 *
		 * @参数 fclCollisionObj fcl碰撞对象
		 * @参数 pointCnt 采点数量
		 * @返回值 生成的json字符串
		 */
		static std::vector<std::string> sampleFclModelToJson(std::vector< fcl::CollisionObject<double>*> fclCollisionObjs, int pointCnt);

	};
}
