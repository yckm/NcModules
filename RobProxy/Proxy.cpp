#include "Proxy.h"
#include <iostream>
#include "../NcParseLib/Parser.h"
#include "glog/logging.h"
#include "Utils/Utils.h"
#include "Utils/Config.h"

#pragma region 变量/常量初始化
namespace mainthread {
	std::shared_mutex req_cmd_mtx;
	std::shared_mutex rob_state_mtx;
	std::shared_mutex batch_mtx;
	std::shared_mutex clash_mutex;

#pragma region 静态常量
	const std::string Proxy::reqTypeGetState = "GetRobState";// 获取机械臂状态
	const std::string Proxy::reqTypePath = "Path";// 执行规划路径
	const std::string Proxy::reqTypeHand = "Hand";// 执行手动控制
	const std::string Proxy::reqTypeSwitch = "Switch";// 手动自动切换
	const std::string Proxy::reqTypeSpeedAdjust = "SpeedAdjust";// 速度调节
	const std::string Proxy::reqTypeClearAllAndStandby = "ClearAllAndStandby";// 清空所有指令并将机器设为就绪状态
	const std::string Proxy::reqTypeShutdown = "Shutdown"; // 关节指令
	const std::string Proxy::reqTypeWysiwyg = "sd:";// 随动指令
#pragma endregion

#pragma region 初始化共享变量
	OptMode Proxy::mode = PATH_MODE; // 默认为自动控制,省的操作
	Cmds* Proxy::cmds = new Cmds();
	WysiwygCmd* Proxy::wysiwygCmd = new WysiwygCmd();
	RobState* Proxy::robState = new RobState();
	int Proxy::arm2ArmDistance = 9999;
	int Proxy::arm2EnvDistanceMm = 9999;
	int Proxy::globalSpeedRate1 = 10;
	int Proxy::globalSpeedRate2 = 10;

	// 当前批次的执行状态
	int Proxy::CURBATCHNO = -1;
	int Proxy::CURBATCHTERMINATED = 0;// 0 未终结 1 已终结

#pragma endregion
#pragma endregion

	/**
	 * @创建人:dnp
	 * @简述:深拷贝cmd
	 * @参数:cmds
	 * @参数:targetCmds
	 * **/
	void CopyCmds(Cmds* targetCmds) {
		targetCmds->RobId = Proxy::cmds->RobId;
		targetCmds->batchNo = Proxy::cmds->batchNo;
		targetCmds->reqType = Proxy::cmds->reqType;
		targetCmds->param = Proxy::cmds->param;
		targetCmds->paramf = Proxy::cmds->paramf;
		targetCmds->terminated = false;

		targetCmds->cmds.clear();
		for (auto& c : Proxy::cmds->cmds) {
			Cmd cmd;
			cmd.name = c.name;
			cmd.tool = c.tool.empty() ? "default" : c.tool;
			cmd.workpiece = c.workpiece.empty() ? "default" : c.workpiece;
			cmd.params = c.params;
			cmd.a = c.a;
			cmd.v = c.v;	
			targetCmds->cmds.push_back(cmd);
		}
	}

#pragma region http 服务
	/**
	 * @创建人:dnp
	 * @简述:解析用户请求参数,该方法只是设置参数,不对机器人进行控制
	 * @参数:body 用户请求消息体
	 * **/
	Resp Proxy::ParseUserRequestAndDistribute(const Cmds* usrCmds)
	{
		Resp  resp{ 1,"Success" };

#pragma region 不使用batchNo的类型(即,不担心重复发送指令的指令集)
		// 切换手/自动 指令
		if (usrCmds->reqType == reqTypeSwitch) {
			if (usrCmds->param == (int)HAND_MODE) {
				mode = HAND_MODE;
			}
			else if (usrCmds->param == (int)WYSIWYG_MODE) {
				mode = WYSIWYG_MODE;
			}
			else{
				mode = PATH_MODE;
			} //0-手动 1-路径
			LOG(INFO) << "mode = > " << (int)mode << std::endl;
			return resp;
		}

		// 设置速率
		if (usrCmds->reqType == reqTypeSpeedAdjust) {
			std::unique_lock<std::shared_mutex> lock(req_cmd_mtx);// 写锁
			if (usrCmds->RobId == 1) {
				globalSpeedRate1 = usrCmds->param;
			}
			else {
				globalSpeedRate2 = usrCmds->param;
			}
			return resp;
		}
#pragma endregion

		// 指令重复
		if (usrCmds->batchNo == cmds->batchNo) {
			resp.status = 0;
			resp.msg = "Instruction repetition";
			return resp;
		}
				
		// 设置当前批次指令执行状态
		{
			std::unique_lock<std::shared_mutex> lk(batch_mtx);// 批次状态写锁
			CURBATCHNO = usrCmds->batchNo; // 设置当前执行指令集
			CURBATCHTERMINATED = 0; // 设置状态未未执行状态
			lk.unlock();
		}

		// 随动控制指令
		if (usrCmds->reqType.compare(0, reqTypeWysiwyg.size(), reqTypeWysiwyg) == 0) {
			return ParseAndDistributeWysiwyg(usrCmds->RobId, usrCmds->reqType, usrCmds->batchNo);
		}

		if (usrCmds->reqType == reqTypeClearAllAndStandby || usrCmds->reqType == reqTypeShutdown || (mode == PATH_MODE && usrCmds->reqType == reqTypePath) || (mode == HAND_MODE && usrCmds->reqType == reqTypeHand)) {
			std::unique_lock<std::shared_mutex> lock(req_cmd_mtx);// 写锁

			cmds->cmds.clear(); // 清空指令集

			cmds->RobId = usrCmds->RobId;
			cmds->batchNo = usrCmds->batchNo;
			cmds->reqType = usrCmds->reqType;
			cmds->param = usrCmds->param;
			cmds->paramf = usrCmds->paramf;
			cmds->terminated = false;

			for (auto& c : usrCmds->cmds) {
				Cmd cmd;
				cmd.name = c.name;
				cmd.tool = c.tool;
				cmd.workpiece = c.workpiece;
				cmd.params = c.params;
				cmd.state = EnumCmdState::ECS_NotStart;
				cmd.v = c.v;
				cmd.a = c.a;
				cmds->cmds.push_back(cmd);
			}

			return  resp;
		}

		resp.status = 0;
		resp.msg = "Not effective";
		return resp;
	}

	/**
	* @创建人 dnp
	* @简介 解析和分配随动控制指令,这部需要分配指令到指定的机械臂
	* @参数 s 随动指令 ( sd:type:cameraName:x:y )
	* @参数 batchNo 批次号
	* @返回值 是否成功
	*/
	Resp Proxy::ParseAndDistributeWysiwyg(int moveRobId, std::string s,int batchNo)
	{
		try {
			const std::string spliter = ":";
			std::vector<std::string> ps = Utils::Utils::split(s, spliter);
			std::string cameraName = ps[1];
			double deltaX = std::stod(ps[2]);
			double deltaY = std::stod(ps[3]);
			int timeoutMs = std::stoi(ps[4]);
			int deltaAsDirection = std::stoi(ps[5]);
			
			{
				std::unique_lock<std::shared_mutex> lock(req_cmd_mtx);// 写锁
				wysiwygCmd->RobId = moveRobId;
				wysiwygCmd->cameraName = cameraName;
				wysiwygCmd->deltaX = deltaX;
				wysiwygCmd->deltaY = deltaY;
				wysiwygCmd->batchNo = batchNo;
				wysiwygCmd->timeoutMs = timeoutMs;
				wysiwygCmd->deltaAsDirection = (deltaAsDirection == 1);
				wysiwygCmd->state = ECS_NotStart;				
			}
			Resp resp{ 1,"Success" };
			return resp;
		}
		catch (const char* e) {
			LOG(INFO) << s << " e:"<<  e;
			Resp resp{ 0,"Fail" };
			return resp;
		}		
	}

	/**
	 * @创建人:dnp
	 * @简述:更新机器人状态
	 * @参数:机械臂状态
	 * **/
	void Proxy::UpdateRobState(int robId, EnumRobState p_robState, EnumProgramState robProgramState, std::vector<float> joints, std::vector<float> tcps, EnumSafetyState safetyState, int errCode,int clashSign,int clashArm) {
		std::unique_lock<std::shared_mutex> lock(rob_state_mtx);// 写入状态
		if (robId == 1) {
			robState->rob1State = p_robState;
			robState->rob1Joints = joints;
			robState->rob1Tcps = tcps;
			robState->rob1ProgramState = robProgramState;
			robState->rob1SafetyState = safetyState;
			robState->rob1ErrCode = errCode;
			robState->rob1ClashSign = clashSign;
			robState->rob1ClashArm = clashArm;
		}
		else {
			robState->rob2State = p_robState;
			robState->rob2Joints = joints;
			robState->rob2Tcps = tcps;
			robState->rob2ProgramState = robProgramState;
			robState->rob2SafetyState;
			robState->rob2ErrCode = errCode;
			robState->rob2ClashSign = clashSign;
			robState->rob2ClashArm = clashArm;
		}
	}

	/**
	 * @创建人:dnp
	 * @创建日期:2023-10-18
	 * @简述: 获取双臂状态
	 * **/
	std::string  Proxy::GetRobState() {
		std::shared_lock<std::shared_mutex> lock(rob_state_mtx); // 读取状态

		json body;
		body["s1"] = (int)robState->rob1State;
		body["p1"] = (int)robState->rob1ProgramState;
		body["j1"] = robState->rob1Joints;
		body["t1"] = robState->rob1Tcps;

		body["s2"] = (int)robState->rob2State;
		body["p2"] = (int)robState->rob2ProgramState;
		body["j2"] = robState->rob2Joints;
		body["t2"] = robState->rob2Tcps;

		auto dists = GetClashDistance();
		body["a2a"] = std::get<0>(dists); // 碰撞距离
		body["a2e"] = std::get<1>(dists);

		body["m"] = (int)mode;
		body["sp1"] = globalSpeedRate1;
		body["sp2"] = globalSpeedRate2;
		body["sf1"] = (int)robState->rob1SafetyState;
		body["sf2"] = (int)robState->rob2SafetyState;
		body["bn"] = CURBATCHNO;
		body["bnt"] = CURBATCHTERMINATED;

		body["err1"] = robState->rob1ErrCode;
		body["err2"] = robState->rob2ErrCode;

		body["c1s"] = robState->rob1ClashSign;
		body["c2s"] = robState->rob2ClashSign;

		body["c1a"] = robState->rob1ClashArm;
		body["c2a"] = robState->rob2ClashArm;


		return body.dump();
	}

	/**
	* @创建人:dnp
	* @创建日期:2023-11-9
	* @简述: 获取机械臂关节角度
	* @返回值: [1关节角度,2关节角度]
	* **/
	std::vector<std::vector<float>> Proxy::GetJoints() {
		std::shared_lock<std::shared_mutex> lock(rob_state_mtx); // 读取状态
		std::vector<std::vector<float>> js = { robState->rob1Joints ,robState->rob2Joints };
		return js;
	}

	void Proxy::CopyWysiwygCmd(WysiwygCmd* sdCmd) {
		// 如果随动指令正在运行发话,新来的指令和当前指令相同情况下,续一下时间即可
		if (sdCmd->state == ECS_Running) {
			if (sdCmd->RobId == wysiwygCmd->RobId and  sdCmd->cameraName == wysiwygCmd->cameraName and sdCmd->deltaX == wysiwygCmd->deltaX and sdCmd->deltaY == wysiwygCmd->deltaY) {
				sdCmd->runTs = Utils::Utils::GetTsMs();
				sdCmd->receivedBatchNo = wysiwygCmd->batchNo;
				return;
			}
			
			// 不管哪个要素出现了变换需要终止当前的随动控制
			sdCmd->terminatedNow = true; 			
		}

		sdCmd->receivedBatchNo = wysiwygCmd->batchNo;
		sdCmd->cameraName = wysiwygCmd->cameraName;
		sdCmd->deltaX = wysiwygCmd->deltaX;
		sdCmd->deltaY = wysiwygCmd->deltaY;
		sdCmd->state = ECS_NotStart;
		sdCmd->timeoutMs = wysiwygCmd->timeoutMs;
		sdCmd->deltaAsDirection = wysiwygCmd->deltaAsDirection;


		{
			int observeCamId = Utils::Config::getCamera(wysiwygCmd->cameraName).robId;
			if (observeCamId != wysiwygCmd->RobId) {
				if (observeCamId == 1) {

					for (int i = 0; i < robState->rob1Joints.size(); i++) {
						sdCmd->observeJoints[i] = robState->rob1Joints[i];
					}

				}
				else {
					std::shared_lock<std::shared_mutex> lock(rob_state_mtx); // 读取状态
					for (int i = 0; i < robState->rob2Joints.size(); i++) {
						sdCmd->observeJoints[i] = robState->rob2Joints[i];
					}
				}
			}
		}

	}
	

	/**
	 * @创建人:dnp
	 * @简述:更新碰撞距离 (碰撞预测线程中调用)
	 * @参数:distance 碰撞距离(单位mm)
	 * **/
	void Proxy::UpdateClashDistance(int arm2armDistance, int arm2envDistance)
	{
		std::unique_lock<std::shared_mutex> lk(clash_mutex);// 批次状态写锁
		arm2ArmDistance = arm2armDistance;
		arm2EnvDistanceMm = arm2envDistance;
		lk.unlock();
	}
	/**
	 * @创建人:dnp
	 * @简述:获取碰撞预测距离
	 * @返回 预测碰撞距离(单位mm)
	 * **/
	bool Proxy::WillClash()
	{
		return false;
		// 反馈说是不应该自动停止,二十由用户决定是否停止 因此注释掉以下代码
		//std::shared_lock<std::shared_mutex> lock(clash_mutex); // 读取状态
		//return arm2ArmDistance < 1 || arm2EnvDistanceMm<1; // 小于10mm认为即将相撞
	}

	std::tuple<int, int> Proxy::GetClashDistance() {
		std::shared_lock<std::shared_mutex> lock(clash_mutex); // 读取状态
		std::tuple<int, int> dists{ arm2ArmDistance,arm2EnvDistanceMm };
		return dists;
	}
	/**
	 * @创建人:dnp
	 * @简述:获取指定机械臂的状态
	 * @参数:robId 机械臂ID
	 * @返回值:是否可用
	 * **/
	std::tuple< EnumRobState, EnumSafetyState> Proxy::GetRobState(int robId)
	{
		std::shared_lock<std::shared_mutex> lock(rob_state_mtx); // 读取状态

		if (robId == 1) {
			return std::tuple< EnumRobState, EnumSafetyState>(robState->rob1State, robState->rob1SafetyState);
		}
		return std::tuple< EnumRobState, EnumSafetyState>(robState->rob2State, robState->rob2SafetyState);
	}

	/**
	* @创建人:dnp
	* @简述:设置指定批次号的指令集为结束状态
	* @参数:batchNo 批次号
	* **/
	void Proxy::UpdateBatchTerminated(int batchNo)
	{
		std::unique_lock<std::shared_mutex> lock(batch_mtx);// 批次状态写锁
		if (batchNo == CURBATCHNO) {
			CURBATCHTERMINATED = 1; // 设置状态为终止状态
		}
	}


	/**
	* @创建人:dnp
	* @创建日期:2023-10-18
	* @简述:子线程从主线程更新自己的指令集
	* @参数: subCmd 子线程指令集
	* @参数: sdCmd 子线程随动指令
	* @返回值: <接收当前指令时候的操作模式,机器人ID,速率>
	* **/
	std::tuple< OptMode, int, int> Proxy::UpdateSubCmds(Cmds* subCmd, WysiwygCmd* sdCmd) {
		std::shared_lock<std::shared_mutex> lock(req_cmd_mtx); // 读取状态
		double speed = subCmd->RobId == 1 ? globalSpeedRate1 : globalSpeedRate2;

		if (mode == WYSIWYG_MODE) {
			if (sdCmd->RobId != wysiwygCmd->RobId or sdCmd->batchNo == wysiwygCmd->batchNo) {
				return std::tuple<OptMode, int, int>(mode, wysiwygCmd->RobId, speed);
			}
			CopyWysiwygCmd(sdCmd);
			return std::tuple<OptMode, int, int>(mode, wysiwygCmd->RobId, speed);
		}

		// 非随动
		if (subCmd->RobId != cmds->RobId || subCmd->batchNo == cmds->batchNo) {
			return std::tuple<OptMode, int, int>(mode, cmds->RobId, speed);
		}
		CopyCmds(subCmd);
		return std::tuple<OptMode, int, int>(mode, cmds->RobId, speed);
	}
}