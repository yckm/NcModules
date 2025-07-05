/*****************************************************************
@文件名称:KinematicsWrapper.h
@创建人:张平
@创建日期:2023-10-28
@简述:新松机械臂正逆向求解
@更改历史:
日期:         作者:             简述:
*******************************************************************/

#pragma once
#include <vector>

namespace wysiwyg {
	namespace Kinematrics
	{
		class KinematicsWrapper
		{
		public:
			/**
			 @函数简介:根据关节角度计算出末端的位姿
			 @参数说明:injointsDeg：入参，六个关节的角度，单位度
			 @参数说明:outPose：出参，前三个为位置信息单位是毫米，后三个是转动信息RX,RY,RZ，单位为度
			 @返回值说明:如果injointsDeg的个数为6，始终会返回成功
			 **/
			bool FKinematics(std::vector<double>& injointsDeg, std::vector<double>& outPose);

			/**
			 @函数简介:根据末端位姿，计算出六个关节的角度
			 @参数说明:inPose：入参，前三个为位置信息单位是毫米，后三个是转动信息RX,RY,RZ，单位为度
			 @参数说明:outjointsDeg：出参，最多八组关节的角度，单位度
			 @返回值说明:如果逆解算成功，返回true，否则返回false
			 **/
			bool IKinematics(std::vector<double>& inPose, std::vector< std::vector<double>>& outjointsDeg);

			/**
			 @函数简介:根据末端位姿，计算出六个关节的角度。选最优解的规则是所有关节运动总和最小
			 @参数说明:inPose：入参，前三个为位置信息单位是毫米，后三个是转动信息RX,RY,RZ，单位为度
			 @参数说明:inCurrjointsDeg：入参，当前关节的角度，用来筛选最优解
			 @参数说明:outjointsDeg：出参，一组关节的角度，单位度
			 @返回值说明:如果逆解算成功，返回true，否则返回false
			**/
			bool IKinematics(std::vector<double>& inPose, std::vector<double>& inCurrjointsDeg, std::vector<double>& outjointsDeg);
		};
	}
}