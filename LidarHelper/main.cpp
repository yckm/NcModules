#include <iostream>
#include "LidarHelper.h"
int main()
{
	LidarHelper lh("http://127.0.0.1:8000");

	// 启动雷达
	BasicMsg res1 = lh.start();

	// 关闭雷达
	BasicMsg res2 = lh.close();

	// 获取帧
	bool isCombineFrame = true;
	Pcd pcd = lh.getFrame(isCombineFrame);

	// 获取电子围栏
	Boundies boundary = lh.getBoundies(500);

	// 获取电子围栏圆柱模型
	auto bs = lh.getBoundryCylinder(200);

	std::cout << "Hello World!\n";
}