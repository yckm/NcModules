#pragma once
#include <string>
#include <vector>

#define ROBCLIENTMODELAPI __declspec(dllexport)

namespace RobHelper {
	/**
	 * @创建人:dnp
	 * @创建日期:2023-10-18
	 * @简述:操作模式没了类型(手动,路径)
	 * **/
	enum OptMode { HAND_MODE = 1, PATH_MODE = 2 };

	/**
	 * @创建人:dnp
	 * @创建日期:2023-10-25
	 * @简述:安全控制器状态信息
	 * **/
	enum class EnumSafetyState
	{
		SS_INIT = 0, //初始化
		SS_WAIT = 2, //等待
		SS_CONFIG = 3, //配置模式
		SS_POWER_OFF = 4, //下电状态
		SS_RUN = 5, //正常运行状态
		SS_RECOVERY = 6, //恢复模式
		SS_STOP2 = 7, //Stop2
		SS_STOP1 = 8, //Stop1
		SS_STOP0 = 9, //Stop0
		SS_MODEL = 10, //模型配置状态
		SS_REDUCE = 12, //缩减模式状态
		SS_BOOT = 13, //引导
		SS_FAIL = 14, //致命错误状态
		SS_UPDATE = 15 //更新状态
	};

	/**
	 * @创建人:dnp
	 * @创建日期:2023-10-23
	 * @简述: 指令
	 * **/
	struct Cmd
	{
		// 指令名称,包括(moveJ moveL  moveJpose moveTcp sleep)
		std::string name;

		// 指令参数(角度获取位姿,长度为6)
		std::vector<float> params;

		// 工具坐标系名称
		std::string tool = "";

		// 工件坐标系名称
		std::string workpiece = "";

		// 速度
		double v = 0.1;

		// 加速度
		double a = 0.1;
	};

	/**
	 * @创建人:dnp
	 * @创建日期:2023-10-23
	 * @简述:lua脚本模型
	 * **/
	class ROBCLIENTMODELAPI LuaModel
	{
	public:
		int robId; // 机器人ID
		int batchNo; // 批次号
		std::string	type; // 指令类型
		int param; // 指令
		float paramf = 0.0;// 额外参数,根据上下文不同而异
		std::string luaContent; // lua脚本内容

		/**
		 * @创建人:dnp
		 * @简述:转换为json字符串
		 * @返回值:json字符串
		 * **/
		std::string toJson();
	};

	class ROBCLIENTMODELAPI JsonModel
	{
	private:
		std::string style = "json"; // 指令类型为json数组
	public:
		int robId; // 机器人ID
		int batchNo; // 批次号
		std::string	type; // 指令类型
		int param = 0;	 // 其他指令值
		float paramf = 0.0;// 额外参数,根据上下文不同而异
		std::vector<Cmd> params; // 指令数组

		/**
		 * @创建人:dnp
		 * @简述:转换为json字符串
		 * @返回值:json字符串
		 * **/
		std::string toJson();
	};

	/**
	 * @创建人:dnp
	 * @创建日期:2023-10-25
	 * @简述:机器人状态枚举
	 * **/
	enum class EnumRobState
	{
		SR_Start = 0, //机器人启动
		SR_Initialize = 1, //机器人初始化
		SR_Logout = 2, //机器人退出登陆，暂未使用
		SR_Login = 3, //机器人登陆，暂未使用
		SR_PowerOff = 4, //机器下电
		SR_Disable = 5, //机器人失能
		SR_Enable = 6 //机器人使能
	};

	/**
	 * @创建人:dnp
	 * @创建日期:2023-10-25
	 * @简述:程序运行状态枚举
	 * **/
	enum class EnumProgramState
	{
		SP_Stopped = 0, //程序停止
		SP_Stopping = 1, //程序正在停止中
		SP_Running = 2, //程序正在运行
		SP_Paused = 3, //程序已经暂停
		SP_Pausing = 4, //程序暂停中
		SP_TaskRuning = 5 //手动示教任务执行中
	};

	/**
	 * @创建人:dnp
	 * @创建日期:2023-10-18
	 * @简述:双臂状态和碰撞距离
	 * **/
	struct RobState
	{
		// 返回数据是否正确
		bool isOk;

		// 机械臂1的状态
		EnumRobState state1 = EnumRobState::SR_Start;

		// Nc程序运行状态
		EnumProgramState programState1 = EnumProgramState::SP_Stopped;

		// 机器人关节
		std::vector<float> joints1;

		// 末端坐标
		std::vector<float> tcps1;

		// 机械臂2的状态
		EnumRobState state2 = EnumRobState::SR_Start;

		// Nc程序运行状态
		EnumProgramState programState2 = EnumProgramState::SP_Stopped;

		// 机器人关节
		std::vector<float> joints2;

		// 末端坐标
		std::vector<float> tcps2;

		// 机械臂到机械臂的距离(单位mm,时候说明已经碰撞)
		int arm2ArmDistance = -1;

		// 机械臂到环境的距离(单位 mm, 0的时候说明已经碰撞了)
		int arm2EnvDistance = -1;

		// 机械臂1全局速率
		int speed1;

		// 机械臂2全局速率
		int speed2;

		// 操作模式
		OptMode mode;

		// 机械臂1 安全控制器状态信息
		EnumSafetyState safetyState1 = EnumSafetyState::SS_RUN;

		// 机械臂2 安全控制器状态信息
		EnumSafetyState safetyState2 = EnumSafetyState::SS_RUN;

		// 机器人1错误代码
		int rob1ErrCode;

		// 机器人2 错误代码
		int rob2ErrCode;

		// RobProxy中的最新的指令的批次号
		int batchNo = -1;

		// batcNo对应的任务是否结束
		int batchTerminated = 0;


		// 机械臂1 碰撞信号
		int rob1ClashSign = -1;

		// 机械臂2 碰撞信号
		int rob2ClashSign = -1;

		// 机械臂1 碰撞臂
		int rob1ClashArm = -1;

		// 机械臂2 碰撞臂
		int rob2ClashArm = -1;
	};

	/**
	 * @创建人:dnp
	 * @创建日期:2023-10-24
	 * @简述:服务器返回消息
	 * @返回值:
	 * **/
	class ROBCLIENTMODELAPI Msg
	{
	public:
		// status=0 失败 ;  status=1 成功
		int status;

		// 提示信息
		std::string msg;

		// 实际消息体
		std::string data;

		// 当前指令的批次号
		int batchNo = 0;

		/**
		 * @创建人:dnp
		 * @创建日期:2023-10-25
		 * @简述:构造函数,默认设置为失败
		 * **/
		Msg(int p_status = 0, std::string p_msg = "fail", std::string p_data = "") {
			status = p_status;
			msg = p_msg;
			data = p_data;
		}

		// 是否成功
		bool IsOk() {
			return status == 1;
		}
	};
}