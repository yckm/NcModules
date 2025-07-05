#pragma once
#include <string>
#include <vector>
#include <json.hpp>
#include "Utils/base64/base64.h"

using json = nlohmann::json;

namespace RobHelper {
	/**
	 * @创建人:董新强
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
	};

	/**
	 * @创建人:董新强
	 * @创建日期:2023-10-23
	 * @简述:lua脚本模型
	 * **/
	class LuaModel
	{
	public:
		int robId; // 机器人ID
		int batchNo; // 批次号
		std::string	type; // 指令类型
		int param; // 指令
		float paramf = 0.0;// 额外参数,根据上下文不同而异
		std::string luaContent; // lua脚本内容

		/**
		 * @创建人:董新强
		 * @简述:转换为json字符串
		 * @返回值:json字符串
		 * **/
		std::string toJson() {
			json body;
			body["RobId"] = robId;
			body["batchNo"] = batchNo;
			body["type"] = type;
			body["param"] = param;
			body["paramf"] = paramf;
			body["style"] = "lua";
			body["params"] = base64_encode(luaContent, false);
			return body.dump();
		}
	};

	class JsonModel
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
		 * @创建人:董新强
		 * @简述:转换为json字符串
		 * @返回值:json字符串
		 * **/
		std::string toJson() {
			json body;
			body["RobId"] = robId;
			body["batchNo"] = batchNo;
			body["type"] = type;
			body["param"] = param;
			body["paramf"] = paramf;
			body["style"] = "json";

			std::vector<nlohmann::json_abi_v3_11_2::json> cmds;
			for (const Cmd& c : params) {
				json cmd;
				cmd["name"] = c.name;
				cmd["tool"] = c.tool;
				cmd["workpiece"] = c.workpiece;
				cmd["params"] = c.params;
				cmds.push_back(cmd);
			}
			body["params"] = cmds;

			return body.dump();
		}
	};

	/**
	 * @创建人:董新强
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
	 * @创建人:董新强
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
	 * @创建人:董新强
	 * @创建日期:2023-10-18
	 * @简述:双臂状态和碰撞距离
	 * **/
	struct RobState
	{
		// 返回数据是否正确
		bool isOk;

		// 机械臂1的状态
		EnumRobState rob1State = EnumRobState::SR_Start;

		// Nc程序运行状态
		EnumProgramState rob1ProgramState = EnumProgramState::SP_Stopped;

		// 机器人关节
		std::vector<float> rob1Joints;

		// 末端坐标
		std::vector<float> rob1Tcps;

		// 机械臂2的状态
		EnumRobState rob2State = EnumRobState::SR_Start;

		// Nc程序运行状态
		EnumProgramState rob2ProgramState = EnumProgramState::SP_Stopped;

		// 机器人关节
		std::vector<float> rob2Joints;

		// 末端坐标
		std::vector<float> rob2Tcps;

		// 碰撞距离(单位mm,为0时候说明已经碰撞)
		int clashDistanceMm = -1;
	};

	/**
	 * @创建人:董新强
	 * @创建日期:2023-10-24
	 * @简述:服务器返回消息
	 * @返回值:
	 * **/
	class Msg
	{
	public:
		// status=0 失败 ;  status=1 成功
		int status;

		// 提示信息
		std::string msg;

		// 实际消息体
		std::string data;

		/**
		 * @创建人:董新强
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