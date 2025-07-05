| 日期 | 描述 | 修改人 |
| --- | --- | --- |
| 2024年1月2日 | 添加随动接口描述 | dnp |
|  |  |  |
|  |  |  |

- 该服务为调度服务客户端,提供api给其他模块调用,以控制机械臂
- 包含文件 RobClient.h
- 示例用法example.cpp
- 命名空间: RobHelper
- 路径 NcControlModule\RobClient 
## **一、流程**
```
// 初始化客户端
RobHelper::RobClient rc("http://192.168.6.6:8000");


// 2. ... 调用具体接口...
```
## **二、接口**
```
/**
* @创建人:dnp
* @简述: 发送lua脚本被调度服务
* @参数: 机器人ID (1-主臂,2-从臂)
* @参数:p_luaContent 用户编写的lua脚本内容
* @返回值: 服务器返回消息(status=0失败, status=1 成功)
* **/
Msg SendLua(int robId, std::string& p_luaContent);

/**
* @创建人:dnp
* @简述:发送指令集被调度服务
* @参数: 机器人ID (1-主臂,2-从臂)
* @参数:p_cmdType 指令类型 ("Path","Hand")
* @参数:p_cmds 指令集
* @返回值: 服务器返回消息(status=0失败, status=1 成功)
* **/
Msg SendCmds(int robId, std::string p_cmdType, std::vector<Cmd>& p_cmds);

/**
* @创建人:dnp
* @简述:该指令控制机械臂每个关节按照给定的速度一直运动，函数执行后会直接运行后续指令。运行speedj 函数后，机械臂会持续运动并忽略后续运动指令，直到接收到speed_stop()函数后停止。
* @参数: 机器人ID (1-主臂,2-从臂)
* @参数:joints_v_list 每个关节的速度，单位rad/s。
* @返回值:服务器返回消息(status=0失败, status=1 成功)
* **/
Msg speedJ(int robId, std::vector<float>  joints_v_list);

/**
* @创建人:dnp
* @简述:该指令控制机械臂末端按照给定的速度一直运动，函数执行后会直接运行后续指令。运行speedl 函数后，机械臂会持续运动并忽略后续运动指令，直到接收到speed_stop()函数后停止。
* @参数: 机器人ID (1-主臂,2-从臂)
* @参数:tcp_v_List 末端速度向量，线速度单位m/s,角速度单位rad/s。
* @返回值: 服务器返回消息(status=0失败, status=1 成功)
* **/
Msg speedL(int robId, std::vector<float> tcp_v_List);

/**
* @创建人:dnp
* @简述:停止speed运动
* @参数: 机器人ID (1-主臂,2-从臂)
* @返回值:服务器返回消息(status=0失败, status=1 成功)
* **/
Msg speedStop(int robId);

/**
* @创建人:dnp
* @简述:设置全局速率
* @参数: 机器人ID (1-主臂,2-从臂)
* @参数:speed 速度比率
* @返回值:服务器返回消息(status=0失败, status=1 成功)
* **/
Msg AdjustSpeed(int robId, float speed);

/**
* @创建人:dnp
* @简述: 设置操作模式为手控模式(切换后,可执行 speed开头的控制)
* @参数: 机器人ID (1-主臂,2-从臂)
* @返回值: 服务器返回消息(status=0失败, status=1 成功)
* **/
Msg SwitchHand(int robId);

/**
* @创建人:dnp
* @简述:设置操作模式为执行脚本/指令集模式
* @参数: 机器人ID (1-主臂,2-从臂)
* @返回值:服务器返回消息(status=0失败, status=1 成功)
* **/
Msg SwitchPath(int robId);

/**
* @创建人:dnp
* @简述:获取两个机械臂的状态
* @参数: 机器人ID (1-主臂,2-从臂)
* @返回值: 双臂状态
* **/
RobState GetState();

/**
* @创建人:dnp
* @简述:关闭机械臂(关闭NC系统,给机械臂断电)
* @参数: 机器人ID (1-主臂,2-从臂)
* @返回值:服务器返回消息(status=0失败, status=1 成功)
* **/
Msg ShutDown(int robId);

/**
* @创建人:dnp
* @简述:使机器人进入就绪状态(不管机器人处于断电,断使能,运行,暂停状态)
* @参数: 机器人ID (1-主臂,2-从臂)
* @返回值:服务器返回消息(status=0失败, status=1 成功)
* **/
Msg ClearAllAndStandby(int robId);

/**
 * @创建人:dnp
 * @简述: 设置环境模型.每次调用,将重新获取环境模型并更新.
 * @参数: safeDistanceMm 指定安全相间距(单位mm)
 * @返回值: 服务器返回消息(status=0失败, status=1 成功)
 * **/
Msg UpdatePcd(int safeDistanceMm);

/**
 * @创建人 dnp
 * @简介 随动控制(手眼同臂)
 * @参数 cameraName 相机名称
 * @参数 deltax x方向增量
 * @参数 deltay y方向增量
 * @返回值 
 */
Msg WysiwygSame(std::string cameraName, float deltax, float deltay);

/**
 * @创建人 dnp
 * @简介 随动控制(手眼异臂)
 * @参数 cameraName 相机名称
 * @参数 deltax x方向增量
 * @参数 deltay y方向增量
 * @返回值
 */
Msg WysiwygDiff(std::string cameraName, float deltax, float deltay);
```
## **三 **、**随动控制调用**
(2024年1月2日)
随动控制为通过观测坐标系控制机械臂在观测坐标系内进行移动.观测坐标系没有深度概念,即便如此,机械臂末端实际移动却是在三维空间中进行移动.
随动控制由以下两种情况组成:

- 手眼同臂: 观测摄像头安装在移动机械臂末端或末端工器具上
- 手眼异臂: 观测摄像头和机械臂分别在不同臂上.该种情况下,摄像头观测另一只臂的法兰平面效果会更好.
- 参数中的 cameraName 为相机名称,对应的相机参数已经配置在 RobProxy 中.配置信息如下:
```
{
	"Main2AssistMat":{
		"type":"4x4",
		"comment":"主臂基座坐标系变换到从臂",
		"value":[...],
		"armId":0
	},
	"AssistMat2Main":{
		"type":"4x4",
		"comment":"从臂基座坐标系变换到主臂",
		"value":[...],
		"armId":0
	},
	"CameraInMainTcp":{
		"type":"4x4",
		"comment":"主臂末端相机",
		"value":[...],
		"armId":1
	},
	"CameraInAssistTcp":{
		"type":"4x4",
		"comment":"从臂末端相机",
		"value":[...],
		"armId":2	
	}
}
```
使用时指定相机名称,也就相当于指定了机械臂.
例如,当我们需要以主臂法兰内摄像头作为观测摄像头,做手眼同臂的运动时,cameraName 的值为CameraInMainTcp

- 对应增量deltax,deltay单位都为 m(米). 在图像上点击进行该操作时,需要将像素按照一定的比率进行转换,比如所 1 像素=0.0001m.具体的比率系数建议设置为可调以适应不同的情境.
- 随动控制控制机械臂行为的具体实现为通过轨迹池实现,本质上是一系列连续的 moveJ 构成.和现有手动控制使用的 speedJ 不同.
```
/**
 * @创建人 dnp
 * @简介 随动控制(手眼同臂)  
 * @参数 cameraName 相机名称
 * @参数 deltax 观测坐标系内x方向增量(该增量的单位为m)
 * @参数 deltay 观测坐标系内y方向增量(该增量的单位为m)
 * @返回值
 */
Msg WysiwygSame(std::string cameraName, float deltax, float deltay);

/**
 * @创建人 dnp
 * @简介 随动控制(手眼异臂) 
 * @参数 cameraName 相机名称
 * @参数 deltax 观测坐标系内x方向增量(该增量的单位为m)
 * @参数 deltay 观测坐标系内y方向增量(该增量的单位为m)
 * @返回值
 */
Msg WysiwygDiff(std::string cameraName, float deltax, float deltay);
```
## **四、 Example**
```
#include <iostream>
#include <string>
#include "RobClient.h"
#include "Models.h"

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
```
