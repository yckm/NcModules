#include "include/RobClient.h"
#include "Utils/httplib.h"
#include "json.hpp"

using json = nlohmann::json;

namespace RobHelper {
	int RobClient::batchNo = 0;
	static std::mutex mtx;  // 创建互斥锁用于保护共享资源
	int RobClient::GetBatchNo(bool isAdd = true)
	{
		std::lock_guard<std::mutex> lock(mtx);  // 锁定互斥锁
		if (isAdd)
		{
			batchNo++;
		}
		return batchNo;
	}

	Msg RobClient::Post(std::string body)
	{
		Msg msg;
		try
		{
			httplib::Client cli(host);

			//cli.set_connection_timeout(300);

			httplib::Headers headers = {
					{ "content-type", "application/json" }
			};

			// 发送请求
			auto res = cli.Post("/nc/proxy", headers, body, "application/json");
			if (res == nullptr || res->status != 200) {
				msg.status = 0;
				return msg;
			}

			auto resp = json::parse(res->body);
			msg.status = resp["status"];
			msg.data = resp["data"];
			msg.msg = resp["msg"];
			return msg;
		}
		catch (std::exception& e)
		{
			msg.status = 0;
			return msg;
		}
	}

	Msg RobClient::SendLua(int robId, std::string& p_luaContent)
	{
		LuaModel model;
		model.robId = robId;
		model.luaContent = p_luaContent;
		model.type = "Path";
		model.batchNo = RobClient::GetBatchNo();

		// 发送指令
		Msg msg = Post(model.toJson());
		msg.batchNo = model.batchNo;
		return msg;
	}

	Msg RobClient::SendCmds(int robId, std::string p_cmdType, std::vector<Cmd>& p_cmds)
	{
		JsonModel model;
		model.robId = robId;
		model.params = p_cmds;
		model.type = p_cmdType;
		model.batchNo = RobClient::GetBatchNo();

		// 发送指令
		Msg msg = Post(model.toJson());
		msg.batchNo = model.batchNo;
		return msg;
	}

	/**
	 * @创建人:dnp
	 * @简述:将json数组转换为 std::vector<float>
	 * @参数:body json数组
	 * **/
	std::vector<float> JsonArr2Floats(nlohmann::json_abi_v3_11_2::json& jsonArr)
	{
		std::vector<float> vec;
		for (float v : jsonArr)
		{
			vec.push_back(v);
		}
		return vec;
	}

	RobState RobClient::GetState()
	{
		httplib::Client cli(host);
		auto res = cli.Get("/nc/proxy");
		RobState robState;
		if (res == nullptr || res->status != 200) {
			robState.isOk = false;
			return robState;
		}

		auto body = json::parse(res->body);
		int status = body["status"];
		std::string msg = body["msg"];

		if (status == 0) {
			robState.isOk = false;
			return robState;
		}

		std::string data = body["data"];

		auto state = json::parse(data);

		robState.isOk = true;

		// 机械臂1
		robState.state1 = state["s1"];
		robState.programState1 = state["p1"];
		robState.joints1 = JsonArr2Floats(state["j1"]);
		robState.tcps1 = JsonArr2Floats(state["t1"]);
		robState.speed1 = state["sp1"];
		robState.safetyState1 = static_cast<EnumSafetyState>(state["sf1"]);

		// 机械臂2
		robState.state2 = state["s2"];
		robState.programState2 = state["p2"];
		robState.joints2 = JsonArr2Floats(state["j2"]);
		robState.tcps2 = JsonArr2Floats(state["t2"]);
		robState.speed2 = state["sp2"];
		robState.safetyState1 = static_cast<EnumSafetyState>(state["sf2"]);

		robState.arm2ArmDistance = state["a2a"];
		robState.arm2EnvDistance = state["a2e"];
		robState.mode = static_cast<OptMode>(state["m"]);

		robState.batchNo = state["bn"];
		robState.batchTerminated = state["bnt"];

		robState.rob1ErrCode = state["err1"];
		robState.rob2ErrCode = state["err2"];

		robState.rob1ClashSign = state["c1s"];
		robState.rob2ClashSign = state["c2s"];
		robState.rob1ClashArm = state["c1a"];
		robState.rob2ClashArm = state["c2a"];

		return robState;
	}

	JsonModel RobClient::GenSingleCmd(int robId, std::string cmdsType, int param) {
		JsonModel model;
		model.robId = robId;
		model.type = cmdsType;
		model.param = param;

		bool isAddBatchNo = true;
		if (cmdsType == "Switch" || cmdsType == "SpeedAdjust")
		{
			isAddBatchNo = false;
		}
		model.batchNo = RobClient::GetBatchNo(isAddBatchNo);

		Cmd cmd;
		cmd.name = cmdsType;
		model.params.push_back(cmd);

		return model;
	}

	Msg RobClient::ShutDown(int robId)
	{
		JsonModel model = GenSingleCmd(robId, "Shutdown", 0);

		// 发送指令
		Msg msg = Post(model.toJson());
		msg.batchNo = model.batchNo;
		return msg;
	}

	Msg RobClient::ClearAllAndStandby(int robId)
	{
		JsonModel model = GenSingleCmd(robId, "ClearAllAndStandby", 0);

		// 发送指令
		Msg msg = Post(model.toJson());
		msg.batchNo = model.batchNo;
		return msg;
	}

	Msg RobClient::UpdatePcd(int safeDistanceMm)
	{
		JsonModel model = GenSingleCmd(1, "PCD", 0);
		model.param = safeDistanceMm;

		// 发送指令
		Msg msg = Post(model.toJson());
		msg.batchNo = model.batchNo;
		return msg;
	}

	Msg RobClient::SwitchHand(int robId)
	{
		JsonModel model = GenSingleCmd(robId, "Switch", 1);

		// 发送指令
		Msg msg = Post(model.toJson());
		msg.batchNo = model.batchNo;
		return msg;
	}

	Msg RobClient::SwitchPath(int robId)
	{
		JsonModel model = GenSingleCmd(robId, "Switch", 2);

		// 发送指令
		Msg msg = Post(model.toJson());
		msg.batchNo = model.batchNo;
		return msg;
	}

#pragma region 手控

	Msg RobClient::GenHandCmds(int robId, std::vector<float> params, std::string cmdName, int param)
	{
		Cmd cmd;
		cmd.name = cmdName;
		cmd.params = params;
		std::vector<Cmd> cmds = { cmd };

		JsonModel model;
		model.robId = robId;
		model.params = cmds;
		model.param = param;
		model.type = "Hand";
		model.batchNo = RobClient::GetBatchNo();

		// 发送指令
		Msg msg = Post(model.toJson());
		msg.batchNo = model.batchNo;
		return msg;
	}

	Msg RobClient::speedJ(int robId, std::vector<float> joints_v_list)
	{
		return GenHandCmds(robId, joints_v_list, "speedJ", 0);
	}

	Msg RobClient::speedL(int robId, std::vector<float> tcp_v_List)
	{
		return GenHandCmds(robId, tcp_v_List, "speedL", 0);
	}

	Msg RobClient::speedStop(int robId)
	{
		std::vector<float> params;
		return GenHandCmds(robId, params, "speedStop", 0);
	}

	Msg RobClient::AdjustSpeed(int robId, float speed)
	{
		std::vector<float> params;
		auto model = GenSingleCmd(robId, "SpeedAdjust", speed);

		// 发送指令
		Msg msg = Post(model.toJson());
		msg.batchNo = model.batchNo;
		return msg;
	}

#pragma endregion

#pragma region  随动控制

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
	Msg RobClient::Wysiwyg(std::string cameraName, int moveRobId,float deltax, float deltay, int timeoutMs,bool deltaAsDirection) {
		std::string cmd = "sd:"
			+ cameraName + ":"
			+ std::to_string(deltax) + ":"
			+ std::to_string(deltay) + ":"
			+ std::to_string(timeoutMs) + ":"
			+ (deltaAsDirection ? "1" : "0");

		
		auto model = GenSingleCmd(moveRobId, cmd, 0);
		// 发送指令
		Msg msg = Post(model.toJson());
		msg.batchNo = model.batchNo;
		return msg;
	}

	Msg RobClient::Enable(int robId)
	{
		JsonModel model = GenSingleCmd(robId, "Enable", 0);

		// 发送指令
		Msg msg = Post(model.toJson());
		msg.batchNo = model.batchNo;
		return msg;
	}

	Msg RobClient::Disable(int robId)
	{
		JsonModel model = GenSingleCmd(robId, "Disable", 0);

		// 发送指令
		Msg msg = Post(model.toJson());
		msg.batchNo = model.batchNo;
		return msg;
	}

	Msg RobClient::PowerOn(int robId)
	{
		JsonModel model = GenSingleCmd(robId, "PowerOn", 0);

		// 发送指令
		Msg msg = Post(model.toJson());
		msg.batchNo = model.batchNo;
		return msg;
	}

	Msg RobClient::PowerOff(int robId)
	{
		JsonModel model = GenSingleCmd(robId, "PowerOff", 0);

		// 发送指令
		Msg msg = Post(model.toJson());
		msg.batchNo = model.batchNo;
		return msg;
	}

	Msg RobClient::Pause(int robId)
	{
		JsonModel model = GenSingleCmd(robId, "Pause", 0);

		// 发送指令
		Msg msg = Post(model.toJson());
		msg.batchNo = model.batchNo;
		return msg;
	}

	Msg RobClient::Resume(int robId)
	{
		JsonModel model = GenSingleCmd(robId, "Resume", 0);

		// 发送指令
		Msg msg = Post(model.toJson());
		msg.batchNo = model.batchNo;
		return msg;
	}

	Msg RobClient::CollisionDetectionReset(int robId)
	{
		JsonModel model = GenSingleCmd(robId, "CollisionDetectionReset", 0);

		// 发送指令
		Msg msg = Post(model.toJson());
		msg.batchNo = model.batchNo;
		return msg;
	}

#pragma endregion
}