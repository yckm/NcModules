// RobClient.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include <string>
#include "include/RobClient.h"
#include "include/Models.h"

void sleep(int ms) {
	std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

int main()
{
	// 初始化客户端
	RobHelper::RobClient rc("http://192.168.6.6:8000");
	int robId = 1; // 主臂

	std::cout << "----------设置全局速率---------------------------" << std::endl;
	rc.AdjustSpeed(robId, 50);

	// 发送lua脚本
	std::cout << "----------执行Lua脚本----------------------" << std::endl;
	std::string luaContent = "moveJ(0.1,0.1,0.1,0.1,0.1,0.1)\nmoveJ(0,0.7,-2,1.4,0,0)";
	RobHelper::Msg msg = rc.SendLua(robId, luaContent);
	std::cout << msg.IsOk() << std::endl;
	sleep(5000);

	// 获取机器人状态
	std::cout << "----------获取双臂抓状态---------------------------" << std::endl;
	RobHelper::RobState state = rc.GetState();
	std::cout << "state=> " << (int)state.programState1 << std::endl;

	std::cout << "----------切换为手动控制 -------------" << std::endl;
	rc.SwitchHand(robId);

	std::vector<float> vs(6, 0);
	vs[0] = -0.5;
	vs[1] = -0.5;
	vs[2] = 0.5;

	std::cout << "----------执行speedL指令----------------------" << std::endl;
	for (int i = 0; i < 25; i++) {
		sleep(100);
		rc.speedL(robId, vs);
	}

	std::cout << "----------执行speedJ指令---------------------" << std::endl;
	for (int i = 0; i < 25; i++) {
		sleep(100);
		rc.speedJ(robId, vs);
	}

	std::cout << "----------执行speed_stop指令----------------------" << std::endl;
	rc.speedStop(robId);

	std::cout << "----------执行speedj指令----------------------" << std::endl;
	for (int i = 0; i < 25; i++) {
		sleep(100);
		rc.speedJ(robId, vs);
	}

	std::cout << "----------切换为自动(路径)模式 -------------" << std::endl;
	rc.SwitchPath(robId);

	std::cout << "----------设置全局速率---------------------------" << std::endl;
	rc.AdjustSpeed(robId, 70);
	//-----------手动传json到服务器---------------------------------------

	std::cout << "----------执行指令集 -------------" << std::endl;
	RobHelper::Cmd cmd1;
	cmd1.name = "moveJ";
	std::vector<float> p1 = { 0.1,0.1,0.1,0.1,0.1,0.1 };
	cmd1.params = p1;

	RobHelper::Cmd cmd2;
	cmd2.name = "moveJ";
	std::vector<float> p2 = { 0.05,0.2,0.13,0.2,0.25,0. };
	cmd2.params = p2;

	std::vector<RobHelper::Cmd> cmds = { cmd1,cmd2 };

	rc.SendCmds(robId, "Path", cmds);
	sleep(5000);
	//----------关机-------------------------------------
	// rc.ShutDown();

	std::cout << "----------使机器人处于就绪状态 -------------" << std::endl;
	rc.ClearAllAndStandby(robId);

	getchar();

	return 0;
}