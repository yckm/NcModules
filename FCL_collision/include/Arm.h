#pragma once
/*****************************************************************
 * @文件名称:Arm.h
 * @创建人:dnp
 * @创建日期:2023 November 
 * @简述:
*******************************************************************/
#include "Eigen/Dense"
#include <vector>
#include <iostream>
#include "Models.h"

namespace Clash{
	typedef Eigen::Matrix< double, 1, 3> Vec3;

	class  Arm
	{
	private:
		double h1, h2, h3, h4, h5, h6, h7, h8; // 每一节臂长

	public:
#pragma region 静态方法
		/**
		 * @创建人:dnp
		 * @简述: 绕x选择theta弧度
		 * @参数:theta 旋转弧度
		 * @返回值: 旋转后的4x4矩阵
		 * **/
		static Eigen::Matrix4d rx(double theta);

		/**
		 * @创建人 dnp
		 * @简介  获取绕x的3x3的旋转矩阵
		 * @参数 弧度
		 * @返回值 3x3旋转矩阵
		 */
		static Eigen::Matrix3d rx3(double theta);
		/**
		 * @创建人:dnp
		 * @简述: 绕y选择theta弧度
		 * @参数:theta 旋转弧度
		 * @返回值: 旋转后的4x4矩阵
		 * **/
		static  Eigen::Matrix4d ry(double theta);

		/**
		 * @创建人 dnp
		 * @简介  获取绕y的3x3的旋转矩阵
		 * @参数 弧度
		 * @返回值 3x3旋转矩阵
		 */
		static Eigen::Matrix3d ry3(double theta);

		/**
		 * @创建人:dnp
		 * @简述: 绕z选择theta弧度
		 * @参数:theta 旋转弧度
		 * @返回值: 旋转后的4x4矩阵
		 * **/
		static Eigen::Matrix4d  rz(double theta);

		/**
		 * @创建人 dnp
		 * @简介  获取绕Z的3x3的旋转矩阵
		 * @参数 弧度
		 * @返回值 3x3旋转矩阵
		 */

		static Eigen::Matrix3d rz3(double theta);

		/**
		 * @创建人:dnp
		 * @简述:平移
		 * @参数:x x方向平移量
		 * @参数:y y方向平移量
		 * @参数:z z方向平移量
		 * @返回值: 平移后的矩阵
		 * **/
		static  Eigen::Matrix4d matMv(double x, double y, double z);

		/**
		 * @创建人:dnp
		 * @简述:获取单位矩阵
		 * @返回值: 单位矩阵
		 * **/
		static Eigen::Matrix4d eye();

		/**
		 * @创建人:dnp
		 * @简述:获取位移
		 * @返回值: 位移(x,y,z)
		 * **/
		static Vec3 getBias(Eigen::Matrix4d& mat);

		/**
		 * @创建人:dnp
		 * @简述:4x4 零矩阵
		 * @返回值: 4x4 零矩阵
		 * **/
		static Eigen::Matrix4d zero();


#pragma endregion

#pragma region 实例方法
		/**
		 * @创建人:dnp
		 * @简述:构造函数
		 * @参数 arms 机械臂长度数组
		 * **/
		Arm(std::vector<double> arms);

		/**
		 * @创建人:dnp
		 * @简述:根据关节角度计算关机的空间坐标
		 * @参数:joints 关节角度
		 * @返回值: 每个关机的空间坐标
		 * **/
		std::vector<Eigen::Matrix4d> GetArmPose(std::vector<double> joints);
#pragma endregion
	};
}
