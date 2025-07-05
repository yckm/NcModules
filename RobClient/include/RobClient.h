#pragma once
#include "Models.h"
#include <string>
#include <mutex>
#include <exception>

#define ROBCLIENTAPI __declspec(dllexport)

namespace RobHelper {
	class ROBCLIENTAPI RobClient
	{
	private:
#pragma region 私有
		std::string host; // 连接地址 ,例如 http://192.168.6.2:80
#pragma region 批次号
		static int batchNo; // 批次号 (全局唯一)
		static int GetBatchNo(bool isAdd); // 获取批次号

		JsonModel GenSingleCmd(int robId, std::string cmdsType, int param); // 执行不带cmds的指令
#pragma endregion
		/**
		 * @创建人:dnp
		 * @简述: http post
		 * @参数:body post body
		 * @返回值: 服务器防护消息
		 * **/
		Msg Post(std::string body);

		/**
		 * @创建人:dnp
		 * @简述:生成手控指令集
		 * @参数:params 参数数组
		 * @参数:cmdName 指令名称
		 * @参数:param 参数值
		 * @返回值: 服务器防护消息
		 * **/
		Msg GenHandCmds(int robId, std::vector<float> params, std::string cmdName, int param = 0);
#pragma endregion

	public:

		/**
		 * @创建人:dnp
		 * @简述:初始化batchNo(调试用)
		 * @参数:p_batchNo
		 * **/
		static void InitBatchNo(int p_batchNo) {
			batchNo = p_batchNo;
		}

		/**
		 * @创建人:dnp
		 * @简述:设置指令要发送目标的uri
		 * @参数:p_host 地址
		 * **/

		RobClient(std::string p_host) {
			host = p_host;
		}

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
		 * @简述: 设置环境pcd
		 * @参数: safeDistanceMm 相间距
		 * @返回值: 服务器返回消息(status=0失败, status=1 成功)
		 * **/
		Msg UpdatePcd(int safeDistanceMm);

		/**
		 * @创建人 dnp
		 * @简介 随动控制(手眼同臂) 具体参考 https://alidocs.dingtalk.com/i/nodes/MNDoBb60VLrZkzgMF3xm2ABK8lemrZQ3
		 * @参数 cameraName 相机名称
		 * @参数 moveRobId 需要运动的机械臂Id(1-主臂,2-从臂)
		 * @参数 deltax 观测坐标系内x方向增量(该增量的单位为m)
		 * @参数 deltay 观测坐标系内y方向增量(该增量的单位为m)
		 * @参数 timeoutMs 超时时间(即即使没有到达目标,也在该时间内停止) 毫秒 . 0表示不限制
		 * @参数 deltaAsDirection 增量作为方向向量,目标位置不再是由deltax和delta有指定的点,而是有二者所指的方向.
		 * @返回值
		 */
		Msg Wysiwyg(std::string cameraName, int moveRobId, float deltax, float deltay,int timeoutMs,bool deltaAsDirection=false);

		/**
		 * @创建人:dnp
		 * @简述: 使能
		 * @参数: 机器人ID (1-主臂,2-从臂)
		 * @返回值:服务器返回消息(status=0失败, status=1 成功)
		 * **/
		Msg Enable(int robId);

		/**
		 * @创建人:dnp
		 * @简述: 失能
		 * @参数: 机器人ID (1-主臂,2-从臂)
		 * @返回值:服务器返回消息(status=0失败, status=1 成功)
		 * **/
		Msg Disable(int robId);

		/**
		 * @创建人:dnp
		 * @简述: 上电
		 * @参数: 机器人ID (1-主臂,2-从臂)
		 * @返回值:服务器返回消息(status=0失败, status=1 成功)
		 * **/
		Msg PowerOn(int robId);

		/**
		 * @创建人:dnp
		 * @简述: 下电
		 * @参数: 机器人ID (1-主臂,2-从臂)
		 * @返回值:服务器返回消息(status=0失败, status=1 成功)
		 * **/
		Msg PowerOff(int robId);

		
		/**
		 * @创建人:dnp
		 * @简述: 暂停
		 * @参数: 机器人ID (1-主臂,2-从臂)
		 * @返回值:服务器返回消息(status=0失败, status=1 成功)
		 * **/
		Msg Pause(int robId);

		
		/**
		 * @创建人:dnp
		 * @简述: 继续
		 * @参数: 机器人ID (1-主臂,2-从臂)
		 * @返回值:服务器返回消息(status=0失败, status=1 成功)
		 * **/
		Msg Resume(int robId);


		
		/**
		 * @创建人:dnp
		 * @简述: 重置碰撞检测
		 * @参数: 机器人ID (1-主臂,2-从臂)
		 * @返回值:服务器返回消息(status=0失败, status=1 成功)
		 * **/
		Msg CollisionDetectionReset(int robId);
	};
}