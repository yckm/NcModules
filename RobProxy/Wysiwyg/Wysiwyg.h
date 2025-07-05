#pragma once
/*****************************************************************
 * @文件名称:Node.h
 * @创建人:dnp
 * @创建日期:2023-11-8
 * @简述:所及即所得控制
 *
 * @更改历史:
 * 日期: 2023年11月8日 作者: dnp 简述: 创建
 *
*******************************************************************/
#include <cmath>
#include <Eigen/Core>
#include "../Utils/Config.h"
#include "Kinematrics/KinematicsWrapper.h"
#include "glog/logging.h"


namespace wysiwyg {

#pragma region 矩阵变换
#define Pi  3.14159265358979323846
	inline Eigen::Matrix4d rx4(double theta)
	{
		Eigen::Matrix4d m;
		m << 1, 0, 0, 0,
			0, cos(theta), -sin(theta), 0,
			0, sin(theta), cos(theta), 0,
			0, 0, 0, 1;
		return m;
	}

	inline static Eigen::Matrix3d rx3(double theta)
	{
		Eigen::Matrix3d m;
		m << 1, 0, 0,
			0, cos(theta), -sin(theta),
			0, sin(theta), cos(theta);
		return m;
	}



	inline static Eigen::Matrix4d ry4(double theta)
	{
		Eigen::Matrix4d m;
		m << cos(theta), 0, sin(theta), 0,
			0, 1, 0, 0,
			-sin(theta), 0, cos(theta), 0,
			0, 0, 0, 1;
		return m;
	}

	inline static Eigen::Matrix3d ry3(double theta)
	{
		Eigen::Matrix3d m;
		m << cos(theta), 0, sin(theta),
			0, 1, 0,
			-sin(theta), 0, cos(theta);
		return m;
	}

	inline static Eigen::Matrix4d rz4(double theta)
	{
		Eigen::Matrix4d m;
		m << cos(theta), -sin(theta), 0, 0,
			sin(theta), cos(theta), 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;
		return m;
	}

	inline static Eigen::Matrix3d rz3(double theta)
	{
		Eigen::Matrix3d m;
		m << cos(theta), -sin(theta), 0,
			sin(theta), cos(theta), 0,
			0, 0, 1;
		return m;
	}

	inline static Eigen::Matrix4d move(double dx, double dy, double dz)
	{
		Eigen::Matrix4d m;
		m << 1, 0, 0, dx,
			0, 1, 0, dy,
			0, 0, 1, dz,
			0, 0, 0, 1;
		return m;
	}

	inline static Eigen::Matrix4d trans4(Eigen::Matrix3d& r, double a, double b, double c) {
		Eigen::Matrix4d mat;
		mat << r(0, 0), r(0, 1), r(0, 2), a,
			r(1, 0), r(1, 1), r(1, 2), b,
			r(2, 0), r(2, 1), r(2, 2), c,
			0, 0, 0, 1;
		return mat;
	}

	/**
	 * @创建人 dnp
	 * @简介 根据位姿获取转换矩阵
	 * @参数 tcp (x,y,z,rx,ry,rz)
	 * @返回值 4x4的转换矩阵
	 */
	static Eigen::Matrix4d tcpPose2mat(std::vector<double>& tcp) {
		Eigen::Matrix3d mat_tcp_in_base = rz3(tcp[5]) * ry3(tcp[4]) * rx3(tcp[3]); // tcp的旋转矩阵
		return trans4(mat_tcp_in_base, tcp[0], tcp[1], tcp[2]);
	}

	static void print3v(std::string tip, Eigen::Vector4d v) {
		std::cout << tip << " => { " << v[0] << " , " << v[1] << " , " << v[2] << " }" << std::endl;
	}
	static void print3v(std::string tip, Eigen::Vector3d v) {
		std::cout << tip << " => { " << v[0] << " , " << v[1] << " , " << v[2] << " }" << std::endl;
	}

	static void print3v(std::string tip, Eigen::Matrix4d mat) {
		std::cout << tip << " => { " << mat(0, 3) << " , " << mat(1, 3) << " , " << mat(2, 3) << " }" << std::endl;
	}

	static void print4d(std::string tip, Eigen::Matrix4d mat) {
		std::cout << "-------------------------------------------------------" << std::endl;
		std::cout << tip << ":" << std::endl;
		for (int i = 0; i < 4; i++) {
			std::cout << "    [";
			for (int j = 0; j < 4; j++) {
				std::cout << mat(i, j) << " , ";
			}
			std::cout << "]," << std::endl;
		}
	}

	static Eigen::Matrix3d getR(Eigen::Matrix4d mat) {
		return mat.block<3, 3>(0, 0);
	}




#pragma endregion
	/**
	 * @创建人 dnp
	 * @简介 随动数据
	 */
	struct WysiwygData {
		// 指令所作用的机器人ID
		int robId;

		// 随动指令集
		std::vector<std::vector<double>> joints;
	};

	class Wysiwyg
	{
	private:
		/**
		 * @创建人 dnp
		 * @简介 根据起始位置,计算两位置直线运动所需要的关节角度列表
		 * @参数 startPoseInBase (基座坐标系下)当前tcp位置
		 * @参数 targetPoseInBase(基座坐标系下)目标tcp位置
		 * @参数 curJoints 当前关节角度
		 * @返回值 运动到目标位置的一系列关节角度列表
		 */
		static std::vector<std::vector<double>> getMoveJoints(std::vector<double> startPoseInBase, std::vector<double> targetPoseInBase, std::vector<double> curJoints);
		
		/**
		 * @创建人 dnp
		 * @简介 计算运动到 tcp下增量所在位置所需要一系列关节的列表
		 * @参数 armTcp 当前默认tcp的位置
		 * @参数 vInTcp tcp下增量向量
		 * @参数 curJoints 当前关节角度
		 * @参数 range 运动边界
		 * @返回值
		 */
		static std::vector<std::vector<double>> calcJointss(std::vector<double>& armTcp, Eigen::Vector4d& vInTcp, std::vector<double>& curJoints, std::vector<double> range);
	
	public:
		/**
		 * @创建人 dnp
		 * @简介 手眼同臂
		 * @参数 cam 观测相机
		 * @参数 xy 相机xy方向上的增量
		 * @参数 armTcp 默认tcp坐标(偏移角度皆为0的那个tcp)的坐标值
		 * @参数 curJoints 当前关节角度(单位弧度)
		 * @参数 range 运动边界
		 * @返回值 一系列关节角度列表
		 */
		static WysiwygData onSameHand(Utils::Camera& cam, std::vector<double> xy, std::vector<double> armTcp, std::vector<double> curJoints,std::vector<double> range);

		/**
		 * @创建人 dnp
		 * @简介 相机在主臂控制从臂运动
		 * @参数 cam 观测相机
		 * @参数 xy 相机xy方向上的增量
		 * @参数 moveArmJoints 运动臂关节角度
		 * @参数 observeArmTcp 观测臂默认tcp当前坐标
		 * @参数 moveArmTcp 运动臂默认tcp当前坐标
		 * @参数 range 运动边界
		 * @返回值 一系列关节角度列表
		 */
		static WysiwygData onDiffHand(Utils::Camera& cam, std::vector<double> xy, std::vector<double> observeArmTcp, std::vector<double> moveArmTcp, std::vector<double> moveArmJoints, std::vector<double> range);


		
		/**
		 * @创建人 dnp
		 * @简介 更加关节角度计算默认tcp位姿
		 * @参数 joints 关节角度(弧度)
		 * @返回值 tcp末端位姿
		 */
		static std::vector<double> getTcpFromJoints(std::vector<double>& joints);
	
	};
}
