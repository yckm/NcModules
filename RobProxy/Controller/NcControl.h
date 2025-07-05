/*****************************************************************
 * @文件名称:NcControl.cpp
 * @创建人:dnp
 * @创建日期:2023-10-18
 * @简述: 机器人控制类,运行在子线程中
 *
 * @更改历史:
 * 日期: 作者: 简述:
 *
*******************************************************************/
#pragma once
#include <string>
#include <vector>
#include <iostream>
#include <chrono>
#include <thread>
#include "../Models.h"
#include "../Cobot/Cobot.h"
#include "../Proxy.h"
#include "../Utils/Utils.h"
#include "glog/logging.h"
#include "../Wysiwyg/Wysiwyg.h"

using utils = Utils::Utils;

namespace Controller {
	/**
	 * @创建人 dnp
	 * @简介 当前正在执行的速度指令
	 */
	struct CurSpeedCmd
	{
		std::vector<double> params; // 指令参数
		long startTime = 0; // 指令开始执行时间
		int batchNo = 0;// 旨在执行指令的batchNo

		/**
		 * @创建人 dnp
		 * @简介 比较两组速度是否相等
		 * @参数 vs 另一组速度
		 * @返回值
		 */
		bool eq(std::vector<float> vs) {
			if (params.empty() || vs.empty()) {
				return false;
			}
			for (int i = 0; i < 6; i++) {
				if (abs(params[i] - vs[i]) > 0.00001) {
					return false;
				}
			}
			return true;
		}

		bool eq(std::vector<double> vs) {
			if (params.empty() || vs.empty()) {
				return false;
			}
			for (int i = 0; i < 6; i++) {
				if (abs(params[i] - vs[i]) > 0.00001) {
					return false;
				}
			}
			return true;
		}
	};

	class NcControl
	{
	private:
		// 新松机器人实例
		Cobot::Cobot* bot;

		// 机器人ID
		int RobId;

		// Nc的IP
		std::string Ip;

		// Nc的端口
		int Port;

		// 指令集
		Cmds* cmds;

		//随动控制指令
		WysiwygCmd* wysiwygCmd;

		//随动指令运动边界(安全)
		std::vector<double> wysiwyRange;



		// 全局速率
		int globalSpeedRate;

#pragma region 上一cmd的状态
		// 上一个cmd的batchNo
		int lastBatchNo = -1;

		// 上一个cmd执行模式
		OptMode lastMode = PATH_MODE;

		// 上一个cmd是否执行完成
		bool lastTerminated = true;

		// 当前在NC执行的任务ID
		int ncTaskId = -1;

		// 当前站在执行的cmd 用于进行比对,如果指令相同的话,就不用终止正在执行的指令
		CurSpeedCmd* curSpeedCmd;

#pragma endregion

	public:
#pragma region 构造函数
		/**
		 * @创建人:dnp
		 * @简述:构造函数
		 * @参数:pRobId 机器人ID
		 * @参数:pIp 机器人IP
		 * @参数:pPort 机器人端口
		 * **/
		NcControl(int pRobId, std::string pIp, int pPort, double wysiwygMin,double wysiwygMax)
		{
			// 初始化指令集
			RobId = pRobId;
			Ip = pIp;
			Port = pPort;

			this->wysiwyRange = { wysiwygMin,wysiwygMax };

			// 手动控制指令
			curSpeedCmd = new CurSpeedCmd();

			// 自动控制指令
			cmds = new Cmds();
			cmds->RobId = RobId;
			cmds->batchNo = -1;
			globalSpeedRate = 10;

			// 随动控制指令
			wysiwygCmd = new WysiwygCmd();
			wysiwygCmd->RobId = pRobId;
			wysiwygCmd->batchNo = -1;
			wysiwygCmd->observeJoints = { 0,0,0,0,0,0 };

			bot = new Cobot::Cobot(Ip, Port, 0);
			if (!bot->connect()) {
				LOG(ERROR) << "connect bot fail" << std::endl;
			}
			else {
				LOG(INFO) << "connect bot success" << std::endl;
			}
		}

		~NcControl()
		{
		}
#pragma endregion

		// 打印一下指令
		void Echo(std::string cmd, std::vector<double> params) {
			std::string s = cmd + "(";
			for (float v : params) {
				s = s + std::to_string(v) + " , ";
			}
			s = s + ")";
			LOG(INFO) << s;
		}

		/**
		 * @创建人:dnp
		 * @简述:设置指令集为终结状态
		 * **/
		void TermainteCmds(std::string tip)
		{
			if (!tip.empty()) {
				LOG(INFO) << "【TermainteCmds】 " << tip;
			}
			cmds->terminated = true;
			ncTaskId = -1;
			mainthread::Proxy::UpdateBatchTerminated(cmds->batchNo);
		}

		/**
		 * @创建人:dnp
		 * @简述:设置随动指令为结束状态
		 * **/
		void TerminateWysiwygCmd(std::string tip) {
			if (!tip.empty()) {
				LOG(INFO) << "【TerminateWysiwygCmd】 " << tip;
			}
			wysiwygCmd->state = EnumCmdState::ECS_Terminated;
			ncTaskId = -1;
			mainthread::Proxy::UpdateBatchTerminated(wysiwygCmd->batchNo);
		}

		/**
		* @创建人:dnp
		* @简述:运行实例
		* **/
		void Run()
		{
			LOG(INFO) << "机器控制服务:" << RobId << "启动成功";
			int sleepTime = 15;//15 ms
			while (1)
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(sleepTime)); // 睡眠10ms

				std::tuple<OptMode, int, int > newModeAndRobId = mainthread::Proxy::UpdateSubCmds(cmds, wysiwygCmd); // 先更新指令集
				OptMode newMode = std::get<0>(newModeAndRobId);// 当前操作模式

				int newRobId = std::get<1>(newModeAndRobId);// 用户最新请求的机器人的Id
				int speedRate = std::get<2>(newModeAndRobId); // 全局速率

				// 保证同时只有一个机械臂在运动
				if (newRobId != RobId) {
					StopMovement("");
					continue;
				}


				// 将要碰撞或已经碰撞,停止运动
				if (mainthread::Proxy::WillClash()) {
					StopMovement("clash");
					continue;
				}

				// 设置全局速率
				if (globalSpeedRate != speedRate) {
					globalSpeedRate = speedRate;
					bot->speed(globalSpeedRate);
				}

				// 检查操作模式是否发送变化,不管运行不运行都停止一下
				if (lastMode != newMode) {
					std::cout << RobId << "  " << newRobId << std::endl;
					StopMovement("operation mode");
					lastMode = newMode; // 更改模式
				}

				// 随动指令
				if (lastMode == WYSIWYG_MODE) {
					ExecuteWysiwyg();
					continue;
				}

				// 超时终止手动控制(因为之前的速度指令可能是很早之前的,所以先执行停止,但是这里不能 使用continue)
				if (lastMode == OptMode::HAND_MODE && Utils::Utils::GetTsMs() - curSpeedCmd->startTime > 300) {
					StopMovement("expired: " + std::to_string(Utils::Utils::GetTsMs() - curSpeedCmd->startTime));
				}


				// 自动指令
				// 指令集执行完成就结束本次循环了
				if (cmds->terminated) {
					continue;
				}

				// 获取正在执行或者尚未执行的指令集
				Cmd* cmd = GetNextCmd();
				if (cmd == nullptr) {
					TermainteCmds("no cmd");
					continue; //没有指令也就进入下一次循环
				}

				// 执行standy,该指令具有高优先级
				if (cmd->state != EnumCmdState::ECS_Terminated && cmd->name == "ClearAllAndStandby") {
					Execute(cmd, speedRate, cmds->batchNo);
					continue;
				}

				// -----------机械臂未就绪不执行指令-------------
				std::tuple< EnumRobState, EnumSafetyState>  states = mainthread::Proxy::GetRobState(RobId);
				EnumRobState state = std::get<0>(states);
				EnumSafetyState safetyState = std::get<1>(states);

				// 紧急停止,这种情况下,徐奥先下电,再上电,再使能.但这种情况可能如果需要恢复的话,只能让用户手动操作了,否则太危险
				if (safetyState == EnumSafetyState::SS_STOP0 || safetyState == EnumSafetyState::SS_STOP1 || safetyState == EnumSafetyState::SS_STOP2) {

					TermainteCmds("紧急停止导致指令集终止");
					if (lastMode == HAND_MODE) {
						bot->powerOff(true);
						bot->powerOn(true);
						bot->enable(true);
					}
					continue;
				}

				if (state == EnumRobState::SR_Disable) {
					TermainteCmds("失能导致指令集终止");

					// 尝试恢复
					int enableResp = bot->enable(true);
					LOG(ERROR) << "try enable bot =>" << enableResp << std::endl;
					continue;
				}

				// 其他状态旧直接停止了
				if (state != EnumRobState::SR_Start && state != EnumRobState::SR_Enable) {
					TermainteCmds("机械臂其他状态导致指令集终止");
					continue;
				}



				// ----任务状态-----------------------------
				int taskState = bot->getTaskState(ncTaskId);

				// 任务还未开始就执行任务
				if (taskState == -1) {
					Execute(cmd, speedRate, cmds->batchNo);
					continue;
				}

				// 正在运行中
				if (taskState == (int)EnumTaskState::ST_Running) {
					if (lastMode == OptMode::HAND_MODE && curSpeedCmd->eq(cmd->params) && curSpeedCmd->batchNo != cmds->batchNo && (cmd->name == "speedJ" || cmd->name == "speedL")) {
						curSpeedCmd->batchNo = cmds->batchNo;
						curSpeedCmd->startTime = Utils::Utils::GetTsMs();
					}
					continue;
				}

				// 任务错误的话,为避免影响,应该终止任务集执行
				if (taskState == (int)EnumTaskState::ST_Error || taskState == (int)EnumTaskState::ST_Illegal) {
					TermainteCmds("任务出错导致指令集终止");
					continue;
				}

				// 任务执行完成/停止/终止 ,只终止当前指令(而非指令集)
				if (taskState == (int)EnumTaskState::ST_Stopped || taskState == (int)EnumTaskState::ST_Finished || taskState == (int)EnumTaskState::ST_Interrupt) {
					cmd->state = EnumCmdState::ECS_Terminated;
					ncTaskId = -1;
					continue;
				}
			}
		}

		/**
		 * @创建人:dnp
		 * @简述:停止机械臂运动
		 * **/
		void StopMovement(std::string tip)
		{
			if (ncTaskId == -1) {
				return; // 没有任务执行直接返回
			}

			if (lastMode == PATH_MODE || lastMode == WYSIWYG_MODE) {
				bot->stop();
			}
			else {
				bot->speedStop();
			}

			TermainteCmds(tip);
		}

		/**
		* @创建人:dnp
		* @简述:获取正在执行的或者尚未执行的下一条执行指令
		* @返回 : 指令
		* **/
		Cmd* GetNextCmd()
		{
			// 指令集已经执行完成了返回null
			if (cmds->cmds.size() == 0 || cmds->terminated) {
				return nullptr;
			}

			for (int i = 0; i < cmds->cmds.size(); i++) {
				if (cmds->cmds[i].state == EnumCmdState::ECS_Terminated) {
					continue;
				}
				return &cmds->cmds[i];
			}

			return nullptr;
		}

		static std::vector<double> getTcp(std::vector<double> joints) {
			return wysiwyg::Wysiwyg::getTcpFromJoints(joints);
		}
		/**
		 * @创建人 dnp
		 * @简介 执行随动控制
		 */
		void ExecuteWysiwyg() {
			if (wysiwygCmd->state == ECS_Terminated) {
				return;
			}

			// 新的随动指令出现,需要终止正在执行的指令,并重置指令状态
			if (wysiwygCmd->terminatedNow) {
				bot->stop();
				wysiwygCmd->state = ECS_NotStart;
				LOG(INFO) << "新随动指令";
				return;
			}

			if (wysiwygCmd->state == ECS_Running) {
				// 超时停止
				if (wysiwygCmd->timeoutMs > 0 and Utils::Utils::GetTsMs() - wysiwygCmd->runTs > wysiwygCmd->timeoutMs) {
					TerminateWysiwygCmd("随动指令超时终止");
					bot->stop(true);
					return;
				}

				// 任务错误的话,任务执行完成/停止/终止 ,停止
				int taskState = bot->getTaskState(ncTaskId);
				if (taskState == (int)EnumTaskState::ST_Error || taskState == (int)EnumTaskState::ST_Illegal || taskState == (int)EnumTaskState::ST_Stopped || taskState == (int)EnumTaskState::ST_Finished || taskState == (int)EnumTaskState::ST_Interrupt) {
					TerminateWysiwygCmd("随动指令终止" + std::to_string(taskState));
					return;
				}

				return;
			}

			// 尚未开始
			if (wysiwygCmd->state == ECS_NotStart) {
				std::vector<double> xy = { wysiwygCmd->deltaX,wysiwygCmd->deltaY };

				// 将delta变做方向的代替就是将目标点延长至不可能达到的位置,或者是是极限位置即可
				if (wysiwygCmd->deltaAsDirection) {
					double m = sqrt(xy[0] * xy[0] + xy[1] * xy[1]);
					xy[0] = xy[0] / m * 1000;
					xy[1] = xy[1] / m * 1000;
				}

				Utils::Camera cam = Utils::Config::getCamera(wysiwygCmd->cameraName);

				if (wysiwygCmd->RobId == cam.robId) {
					std::vector<double> armJoints = bot->getJoints();
					std::vector<double> armTcp = getTcp(armJoints);

					wysiwyg::WysiwygData js = wysiwyg::Wysiwyg::onSameHand(cam, xy, armTcp, armJoints, wysiwyRange);
					if (js.joints.size() == 0) {
						TerminateWysiwygCmd("随动指令终止,无法解算出关节角度" );
						return;
					}
					ncTaskId = bot->moveJs(js.joints, 1, 2, false);
					std::cout <<"bot->moveJs.same=>" <<  ncTaskId << std::endl;
				}
				else {
					//手眼异臂
					std::vector<double> observeArmTcp = getTcp(getTcp(wysiwygCmd->observeJoints));
					std::vector<double> moveArmJoints = bot->getJoints();
					std::vector<double> moveArmTcp = getTcp(moveArmJoints);

					wysiwyg::WysiwygData js = wysiwyg::Wysiwyg::onDiffHand(cam, xy, observeArmTcp, moveArmTcp, moveArmJoints, wysiwyRange);
					if (js.joints.size() == 0) {
						TerminateWysiwygCmd("随动指令终止,无法解算出关节角度");
						return;
					}

					ncTaskId = bot->moveJs(js.joints, 1, 2, false);
				}
				// 更新随动指令状态
				wysiwygCmd->state = ECS_Running;
				wysiwygCmd->runTs = utils::GetTsMs();
				wysiwygCmd->batchNo = wysiwygCmd->receivedBatchNo;
				wysiwygCmd->terminatedNow = false;

				return;
			}

		}


		/**
		 * @创建人:dnp
		 * @简述:控制机械臂执行指令(这里只管执行,不管状态)
		 * @参数:cmd 指令
		 * @参数:speedRate 速率
		 * @参数:batchNo 批次号
		 * **/
		void Execute(Cmd* cmd, int speedRate, int batchNo)
		{
			if (cmd == nullptr) {
				return;
			}

			//// 设置全局速率
			//if (globalSpeedRate != speedRate) {
			//	globalSpeedRate = speedRate;
			//	bot->speed(globalSpeedRate);
			//}

			// 睡眠
			if (cmd->name == "sleep") {
				std::this_thread::sleep_for(std::chrono::milliseconds((int)cmd->params[0]));
				cmd->state = EnumCmdState::ECS_Terminated;
				ncTaskId = -1;
				return;
			}

#pragma region 更改状态
			ncTaskId = -1;
			cmd->state = EnumCmdState::ECS_Running;
			lastBatchNo = cmds->batchNo;
			lastTerminated = false;
			cmd->runTs = utils::GetTsMs();
#pragma endregion
			double v = cmd->v;
			double a = cmd->a;

			std::vector<double> params;
			for (float v : cmd->params) {
				params.push_back(v);
			}
			if (cmd->name == "moveJ") {
				if (v > 100) {
					v = 100;
				}
				if (a > 50) {
					a = 50;
				}
				ncTaskId = bot->moveJ(params, v, a, false);
				Echo("moveJ", params);
				return;
			}

			if (cmd->name == "moveJ2") {
				if (v > 3.14) {
					v = 3.14;
				}
				ncTaskId = bot->moveJ2(params, v, a, false);
				Echo("moveJ2", params);
				return;
			}

			if (cmd->name == "moveJpose") {
				ncTaskId = bot->movePose(params, cmd->v, cmd->a, cmd->tool, cmd->workpiece, false);
				Echo("moveJpose", params);
				return;
			}

			if (cmd->name == "moveL") {
				if (v > 1) {
					v = 1;
				}
				if (a > 1) {
					a = 1;
				}
				ncTaskId = bot->moveL(params, v, a, cmd->tool, cmd->workpiece, false);
				Echo("moveL", params);
				return;
			}

			if (cmd->name == "moveTcp") {
				ncTaskId = bot->tcpMove(params, cmd->v, cmd->a, cmd->tool, false);
				Echo("moveTcp", params);
				return;
			}

#pragma region speed
			if (cmd->name == "speedJ") {
				if (!curSpeedCmd->eq(params)) {
					bot->speedStop();
				}
				ncTaskId = bot->speedJ(params, 1, 10000, false);
				curSpeedCmd->params = params;
				curSpeedCmd->startTime = Utils::Utils::GetTsMs();
				curSpeedCmd->batchNo = batchNo;
				return;
			}

			if (cmd->name == "speedL") {
				if (!curSpeedCmd->eq(params)) {
					bot->speedStop();
				}
				ncTaskId = bot->speedL(params, 1, 10000, false);
				curSpeedCmd->params = params;
				curSpeedCmd->startTime = Utils::Utils::GetTsMs();
				curSpeedCmd->batchNo = batchNo;
				return;
			}

			if (cmd->name == "speedStop") {
				bot->speedStop();
				ncTaskId = -1;
				cmd->state == EnumCmdState::ECS_Terminated;
				Echo("speedStop", params);
				return;
			}

#pragma endregion

#pragma region 管理



			if (cmd->name == "Shutdown") {
				bot->shutdown();
				Echo("shutdown", params);
				return;
			}

			if (cmd->name == "ClearAllAndStandby") {
				ncTaskId = -1;
				cmd->state = EnumCmdState::ECS_Terminated;
				bot->standby();
				if (lastMode == HAND_MODE) {
					bot->speedStop(true);
				}
				else {
					bot->stop(true);
				}
				Echo("ClearAllAndStandby", params);
				return;
			}

			if (cmd->name == "Enable") {
				bot->enable(false);
				Echo("Enable", params);
				return;
			}

			if (cmd->name == "Disable") {
				bot->disable(false);
				Echo("Disable", params);
				return;
			}

			if (cmd->name == "PowerOn") {
				bot->powerOn(false);
				Echo("PowerOn", params);
				return;
			}

			if (cmd->name == "PowerOff") {
				bot->powerOff(false);
				Echo("PowerOff", params);
				return;
			}

			if (cmd->name == "CollisionDetectionReset") {
				bot->collision_detection_reset();
				Echo("CollisionDetectionReset", params);
				return;
			}

			if (cmd->name == "Pause") {
				bot->pause(false);
				Echo("Pause", params);
				return;
			}

			if (cmd->name == "Resume") {
				bot->resume(false);
				Echo("Resume", params);
				return;
			}


#pragma endregion
		}

	};
}