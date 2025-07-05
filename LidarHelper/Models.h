#pragma once
#include <string>
#include <vector>

/**
 * @创建人:dnp
 * @创建日期:2023-11-3
 * @简述:操作结果消息
 * @返回值:
 * **/
struct BasicMsg
{
	std::string msg; // 失败提示
	int status = 0; // 0- 失败  1-成功
};

/**
 * @创建人:dnp
 * @创建日期:2023-11-3
 * @简述:雷达点云信息
 * **/
struct Pcd {
	std::string msg; // 失败提示
	int status = 0; // 0- 失败  1-成功
	std::vector<float> lower;
	std::vector< std::vector<float>> pcd; // 点云 Nx3
};

/**
 * @创建人:dnp
 * @创建日期:2023-11-3
 * @简述:电子围栏信息
 * **/
struct Boundies {
	std::string msg; // 失败提示
	int status = 0; // 0- 失败  1-成功

	std::vector< std::vector<float>> boundaries; // 电子围栏点云 Nx3
	std::vector< std::vector<float>> env; // 环境点云 Nx3
	std::vector< std::vector< std::vector<float>>> vertices; // 线缆分段obb的八个顶点
	std::vector< std::vector< std::vector<float>>> Xaxis;// 分段线缆obb x轴中线线的点云
};

/**
 * @创建人:dnp
 * @创建日期:2023-11-9
 * @简述:电子围栏边界的圆柱
 * **/
struct BoundryCylinder {
	double radius;
	double height;

	// 变换矩阵 (旋转+仿射)
	std::vector<std::vector<double>> trans;
};
