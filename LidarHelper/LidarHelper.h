#pragma once
#include "Models.h"
#define LIDARHELPERAPI __declspec(dllexport)

class LIDARHELPERAPI LidarHelper
{
private:
	std::string host;

public:
	/**
	 * @创建人:dnp
	 * @简述:构造函数
	 * @参数:p_host 雷达服务的地址( http://ip:port)
	 * **/
	LidarHelper(std::string p_host);

	/**
	 * @创建人:dnp
	 * @简述:启动雷达
	 * @返回值: 操作结果
	 * **/
	BasicMsg start();

	/**
	 * @创建人:dnp
	 * @简述:关闭雷达
	 * @返回值:操作结果
	 * **/
	BasicMsg close();

	/**
	 * @创建人:dnp
	 * @简述:获取单帧点云
	 * @参数 isCombineFrames 是否获取合并后的帧( true 将多帧合并为一帧进行获取, false 获取一帧)
	 * @返回值:单帧点云
	 * **/
	Pcd getFrame(bool isCombineFrames);

	/**
	 * @创建人:dnp
	 * @简述:获取电子围栏信息
	 * @参数:distanceMm 安全相间距(单位 毫米)
	 * @返回值: 边界信息
	 * **/
	Boundies getBoundies(int distanceMm);

	/**
	 * @创建人:dnp
	 * @简述:获取电子围栏的圆柱信息
	 * @参数:distanceMm 安全相间距(单位 毫米)
	 * @返回值:电子围栏的圆柱信息数组
	 * **/
	std::vector<BoundryCylinder> getBoundryCylinder(int distanceMm);
};
