#include "Cobot.h"
#include <iostream>
#include "../Models.h"

namespace Cobot {
	Cobot::Cobot(std::string p_ip, int p_port, int p_channel)
	{
		ip = p_ip;
		port = p_port;
		channel = p_channel;
	}

	Cobot::Cobot::~Cobot()
	{
	}

	bool Cobot::Cobot::connect()
	{
		if (bot != nullptr) {
			delete bot;
		}

		bot = new DucoRPC::DucoCobot(ip, port);// 2.9版本以前不添加DucoRPC
		return bot->open() > -1;
	}

	// 获取非阻塞任务ID
	int Cobot::getTaskState(int taskId)
	{
		if (taskId < 0) {
			return -1;
		}
		return  bot->get_noneblock_taskstate(taskId);;
	}

#pragma region 系统控制
	int Cobot::Cobot::close() {
		return bot->close();
	}

	/// <summary>
	/// 上电
	/// </summary>
	/// <param name="block">是否阻塞(默认阻塞)</param>
	/// <returns></returns>
	int Cobot::Cobot::powerOn(bool block) {
		return bot->power_on(block);
	}

	/// <summary>
	/// 机器人上使能
	/// </summary>
	/// <param name="block">是否阻塞(默认阻塞)</param>
	/// <returns></returns>
	int Cobot::Cobot::enable(bool block) {
		return bot->enable(block);
	}

	/// <summary>
	/// 机器人下电
	/// </summary>
	/// <param name="block">是否阻塞(默认阻塞)</param>
	/// <returns></returns>
	int Cobot::Cobot::powerOff(bool block) {
		return bot->power_off(block);
	};

	/// <summary>
	/// 机器人下使能
	/// </summary>
	/// <param name="block">是否阻塞(默认阻塞)</param>
	/// <returns></returns>
	int Cobot::Cobot::disable(bool block) {
		return bot->disable(block);
	}

	/// <summary>
	/// 让机器人处于就绪状态
	/// </summary>
	/// <param name="block">是否阻塞(默认阻塞)</param>
	/// <returns></returns>
	int Cobot::Cobot::standby() {
		// 查询机相关状态
		std::vector<int8_t> state;
		bot->get_robot_state(state);
		int robState = state[0]; //表示机器人状态
		int programState = state[1]; //表示程序状态
		int safeState = state[2]; //表示安全控制器状态

		// 机械臂不处于安全状态
		if (safeState != (int)EnumSafetyState::SS_RUN) {
			if ((safeState == (int)EnumSafetyState::SS_WAIT || safeState == (int)EnumSafetyState::SS_MODEL) and programState == (int)EnumProgramState::SP_Stopped and robState == (int)EnumRobState::SR_PowerOff) {
				bot->power_on(true);
				bot->enable(true);
				return  1;
			}

			// 保护性停止需要先poweroff 后上电再使能
			if (safeState == (int)EnumSafetyState::SS_STOP0 || safeState == (int)EnumSafetyState::SS_STOP1 || safeState == (int)EnumSafetyState::SS_STOP2) {
				bot->power_off(true);
				bot->power_on(true);
				bot->enable(true);
				return 1;
			}

			return 1;
		}
		//----------------机械臂处于安全状态下-------------------------------------------------------------------------
		// 上电+使能
		if (robState == (int)EnumRobState::SR_PowerOff) {
			bot->power_on(true);// 上电
			bot->enable(true);// 使能
			return 1;
		}

		// 使能
		if (robState == (int)EnumRobState::SR_Disable) {
			bot->enable(true);
			return 1;
		}
		// 暂停/运行状态
		if (programState == (int)EnumProgramState::SP_Paused || programState == (int)EnumProgramState::SP_Running) {
			bot->stop(true);
			return 1;
		}

		return 0;
	}

	/// <summary>
	///
	/// </summary>
	/// <param name="block">是否阻塞(默认阻塞)</param>
	/// <returns></returns>
	int Cobot::Cobot::shutdown() {
		bot->disable(true);
		bot->power_off(true);
		return bot->shutdown(true);
	}
#pragma endregion

#pragma region 任务控制
	/// <summary>
	/// 停止所有任务。
	/// </summary>
	/// <param name="block">是否阻塞(默认阻塞)</param>
	/// <returns></returns>
	int Cobot::Cobot::stop(bool block) {
		return bot->stop(block);
	}
	/// <summary>
	/// 暂停所有任务。
	/// </summary>
	/// <param name="block">是否阻塞(默认阻塞)</param>
	/// <returns></returns>
	int Cobot::Cobot::pause(bool block) {
		return bot->pause(block);
	}
	/// <summary>
	/// 恢复所有暂停的任务。
	/// </summary>
	/// <param name="block">是否阻塞(默认阻塞)</param>
	/// <returns></returns>
	int Cobot::Cobot::resume(bool block) {
		return bot->resume(block);
	}

	/**
	 * @创建人 dnp
	 * @简介 重置碰撞检测警告
	 */
	int Cobot::Cobot::collision_detection_reset() {
		return bot->collision_detection_reset();
	}


#pragma endregion

#pragma region 运动控制
	/// <summary>
	/// 该指令控制机械臂从当前状态，按照关节运动的方式移动到目标关节角状态。
	/// </summary>
	/// <param name="jointLs">数组对应1-6 关节的目标关节角度，单位rad</param>
	/// <param name="v">关节角速度，单位（系统设定速度的百分比%），取值范围（0,100]</param>
	/// <param name="a">关节角加速度，单位（系统设定加速度的百分比%），取值范围（0, 100]</param>
	/// <param name="block">指令是否阻塞型指令，如果为false 表示非阻塞指令，指令会立即返回</param>
	/// <returns>当配置为阻塞执行，返回值代表当前任务结束时的状态，若无融合为Finished, 若有融合为Interrupt。当配置为非阻塞执行，返回值代表当前任务的id。</returns>
	int Cobot::Cobot::moveJ(std::vector<double> jointLs, double v, double a, bool block)
	{
		return bot->movej(jointLs, v, a, 0, block);
	}

	/// <summary>
	/// 该指令控制机械臂从当前状态，按照关节运动的方式移动到目标关节角状态。
	/// </summary>
	/// <param name="jointLs">数组对应1-6 关节的目标关节角度，单位rad</param>
	/// <param name="v">关节角速度，单位（rad/s），取值范围（0,100]</param>
	/// <param name="a">关节角加速度，单位（rad/s2），取值范围（0, 100]</param>
	/// <param name="block">指令是否阻塞型指令，如果为false 表示非阻塞指令，指令会立即返回</param>
	/// <returns>当配置为阻塞执行，返回值代表当前任务结束时的状态，若无融合为Finished, 若有融合为Interrupt。当配置为非阻塞执行，返回值代表当前任务的id。</returns>
	int Cobot::Cobot::moveJ2(std::vector<double> jointLs, double v, double a, bool block)
	{
		return bot->movej2(jointLs, v, a, 0, block);
	}

	/// <summary>
	/// 该指令控制机械臂从当前状态，按照关节运动的方式移动到末端目标位置。
	/// </summary>
	/// <param name="pose">对应末端的位姿，位置单位m，姿态以Rx、Ry、Rz 表示，单位rad</param>
	/// <param name="v">关节角速度，单位（系统设定速度的百分比%），取值范围（0,100]</param>
	/// <param name="a">关节加速度，单位（系统设定加速度的百分比%），取值范围（0,100]</param>
	/// <param name="tool">设置使用的工具的名称,为空时默认为当前使用的工具</param>
	/// <param name="wobj">设置使用的工件坐标系的名称，为空时默认为当前使用的工件坐标系</param>
	/// <param name="block">指令是否阻塞型指令，如果为false 表示非阻塞指令，指令会立即返回。</param>
	/// <returns>当配置为阻塞执行，返回值代表当前任务结束时的状态，若无融合为Finished, 若有融合为Interrupt。当配置为非阻塞执行，返回值代表当前任务的id。</returns>
	int Cobot::Cobot::movePose(std::vector<double> pose, double v, double a, std::string tool, std::string wobj, bool block)
	{
		std::vector<double> q_near;
		return bot->movej_pose(pose, v, a, 0, q_near, tool, wobj, block);
	}

	/// <summary>
	/// 该指令控制机械臂末端从当前状态按照直线路径移动到目标状态。
	/// </summary>
	/// <param name="pose">对应末端的位姿，位置单位m，姿态以Rx、Ry、Rz 表示，单位rad</param>
	/// <param name="v">末端速度，单位（m/s）</param>
	/// <param name="a">末端加速度，单位（m/s2）。</param>
	/// <param name="tool">设置使用的工具的名称，为空时默认为当前使用的工具</param>
	/// <param name="wobj">设置使用的工件坐标系的名称，为空时默认为当前使用的工件坐标系。</param>
	/// <param name="block">指令是否阻塞型指令，如果为false 表示非阻塞指令，指令会立即返回。</param>
	/// <returns>当配置为阻塞执行，返回值代表当前任务结束时的状态，若无融合为Finished, 若有融合为Interrupt。当配置为非阻塞执行，返回值代表当前任务的id。</returns>
	int Cobot::Cobot::moveL(std::vector<double> pose, double v, double a, std::string tool, std::string wobj, bool block)
	{
		std::vector<double> q_near;
		return bot->movel(pose, v, a, 0, q_near, tool, wobj, block);
	}
	/// <summary>
	/// 该指令控制机械臂沿工具坐标系直线移动一个"增量"。
	/// </summary>
	/// <param name="offset_in_tool">表示工具坐标系下的位姿偏移量。</param>
	/// <param name="v">直线移动的速度，单位（m/s），当x、y、z 均为0 时，线速度按比例换算成角速度。</param>
	/// <param name="a">加速度，单位（m/ s2）</param>
	/// <param name="tool">设置使用的工具的名称，为空时默认为当前使用的工具。</param>
	/// <param name="block">指令是否阻塞型指令，如果为false 表示非阻塞指令，指令会立即返回。</param>
	/// <returns>当配置为阻塞执行，返回值代表当前任务结束时的状态，若无融合为Finished, 若有融合为Interrupt。当配置为非阻塞执行，返回值代表当前任务的id。</returns>
	int Cobot::Cobot::tcpMove(std::vector<double> offset_in_tool, double v, double a, std::string tool, bool block)
	{
		std::vector<double> q_near;
		return bot->tcp_move(offset_in_tool, v, a, 0, tool, block);
	}

	/**
	* @brief 样条运动函数,该指令控制机器人按照空间样条进行运动。
	* @param plist (Nx6) 在设置工件坐标系下的末端位姿列表,最多不超过50 个点，格式如下：[p1,p2,p3,…,pi,…]其中pi 为空间位姿，如[0.4,0.5,0.5,1.2,0.717,1.414]。
	* @param v 末端速度，单位（m/s）。
	* @param a 末端加速度，单位（m/s2）。
	* @param tool 设置使用的工具的名称，为空时默认为当前使用的工具。
	* @param wobj 设置使用的工件坐标系的名称，为空默认为当前使用的工件坐标系
	* @param block block : 指令是否阻塞型指令，如果为false 表示非阻塞指令，指令会立即返回。
	*/
	int Cobot::spline(std::vector<std::vector<double>> plist, double v, double a, std::string tool, std::string wobj, bool block)
	{
		return bot->spline(plist, v, a, tool, wobj, block);
	}

#pragma region 速度控制指令
	int Cobot::speedJ(std::vector<double> joints_v_list, double a, int time/*=200 */, bool block/*=false*/)
	{
		return bot->speedj(joints_v_list, a, time, block);
	}

	int Cobot::speedL(std::vector<double> tcp_v_List, double a, int time/*=200*/, bool block/*=false*/)
	{
		return bot->speedl(tcp_v_List, a, time, block);
	}

	int Cobot::speedStop(bool block)
	{
		return bot->speed_stop(block);
	}

	int Cobot::speed(double val)
	{
		if (val < 1) {
			return bot->speed(1);
		}
		if (val > 100) {
			return bot->speed(100);
		}
		return bot->speed(val);
	}
#pragma endregion

#pragma region 轨迹池

	/**
	 * @创建人 dnp
	 * @简介 样条运动函数,该指令控制机器人按照空间样条进行运动。 连续两条spline 一起使用时，需要注意前一条的spline 结束点与后一条的spline起始点，不能是同一个点。
	 * @参数 pointList 在设置工件坐标系下的末端位姿列表,最多不超过50 个点，格式如下：
	 * [p1,p2,p3,…,pi,…] 其中p[i] 为空间位姿，如[0.4,0.5,0.5,1.2,0.717,1.414]。
	 * @参数 pointNum 样条点的个数
	 * @参数 v 末端速度，范围(0, 5]，单位(m/s)
	 * @参数 a 末端加速度，范围(0, ∞]，单位(m/s2)
	 * @参数 tool 设置使用的工具的名称，为空时默认为当前使用的工具。
	 * @参数 wobj 设置使用的工件坐标系的名称，为空默认为当前使用的工件坐标系
	 * @返回值 当配置为阻塞执行，返回值代表当前任务结束时的状态，若无融合为Finished,若有融合为Interrupt。当配置为非阻塞执行，返回值代表当前任务的id 信息，用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态。 
	 */
	int Cobot::spline(double** plist, int pointNum, double v, double a, char* tool, char* wobj, bool block) {
		
		std::vector<std::vector<double>> pts;
		if (v > 2) {
			v = 2;
		}

		if (a > 10) {
			a = 10;
		}

		for (int i = 0; i < pointNum; i++) {
			if (i > 49) {
				break; // 新松这里最大接收50个点
			}
			std::vector<double> pt;
			for (int j = 0; j < 6; j++) {
				pt.push_back(plist[i][j]);
			}
			pts.push_back(pt);
		}

		return bot->spline(pts, v, a, tool, wobj, block);
	}

	/**
	 * @创建人 dnp
	 * @简介 将一组js连续执行
	 * @参数 joints 关节角度列表
	 * @参数 pointNum 关节角度组数
	 * @参数 v 速度 (rad/s)
	 * @参数 a 加速度(rad/s2)
	 * @参数 block 是否阻塞
	 * @返回值 轨迹池当前存量
	 */
	int Cobot::moveJs(std::vector<std::vector<double>> joints,  double v, double a, bool block) {		
		bot->trackClearQueue();		
		bot->trackEnqueue(joints, true);
		return bot->trackJointMotion(v, a, block);
	}

	/**
	 * @创建人 dnp
	 * @简介 该函数将一组points 点位信息输入到机器人控制器中的轨迹池。最大数量为1000 个
	 * @参数 joints 关节角度列表
	 * @参数 pointNum 关节角度组数
	 * @参数 block 是否阻塞
	 * @返回值 轨迹池当前存量
	 */
	int Cobot::trackEnqueue(double** joints, int pointNum, bool clearQueue, bool block) {
		

		if (clearQueue) {
			bot->trackClearQueue();
		}

		std::vector<std::vector<double>> js;

		for (int i = 0; i < pointNum; i++) {
			if (i > 1000) {
				break; // 新松这里最大接收1000个点
			}
			std::vector<double> pt;
			for (int j = 0; j < 6; j++) {
				pt.push_back(joints[i][j]);
			}
			js.push_back(pt);
		}
		int resp = bot->trackEnqueue(js, block);
		if (resp == -1) {
			return 1000;
		}
		return bot->getQueueSize();
	}

	/**
	 * @创建人 dnp
	 * @简介 该函数用于将机器人控制器中的轨迹池清空。
	 * @返回值 阻塞执行，返回值代表当前任务结束时的状态。
	 */
	int Cobot::trackClear() {
		
		return bot->trackClearQueue();
	}

	/**
	 * @创建人 dnp
	 * @简介 该函数用于获取机器人控制器中的当前轨迹池大小。
	 * @返回值  阻塞执行，返回值为当前轨迹池大小。
	 */
	int Cobot::trackSize() {
		
		return bot->getQueueSize();
	}

	/**
	 * @创建人 dnp
	 * @简介 该函数执行时，机器人的各关节将顺序到达轨迹池中的点位值直到轨迹池中无新的点位。执行过程中，主导关节（关节位置变化最大的关节）将以speed 与acc 规划运动，其他关节按比例缩放。注：如果已经开始执行停止规划，将不再重新获取轨迹池中的数据，直到完成停止。停止后如果轨迹池中有新的点位，将重新执行跟随。为保证运动连续性，建议至少保证轨迹池中有10 个数据。
	 * @参数 v 最大关节速度，单位(rad/s）。
	 * @参数 a 最大关节加速度，单位(rad/s2）。
	 * @参数 block 指令是否阻塞型指令，如果为false表示非阻塞指令，指令会立即返回。
	 * @返回值
	 */
	int Cobot::trackMove(double v, double a, bool block) {
		

		if (bot->getQueueSize() < 10) {
			return -1;
		}

		return bot->trackJointMotion(v, a, block);
	}
#pragma endregion


#pragma region 获取数据
	/**
	 * @创建人 dnp
	 * @简介 获取关节角度
	 * @返回值 [j1,j2,j3,j4,j5,j6]
	 */
	std::vector<double> Cobot::getJoints() {
		std::vector<double> joints;
		bot->get_actual_joints_position(joints);
		return joints;
	}

	/**
	 * @创建人 dnp
	 * @简介 获取当前tcp位姿
	 * @返回值 [x,y,z,rx,ry,rz]
	 */
	std::vector<double> Cobot::getTcpPose() {
		std::vector<double> pos;
		bot->get_tcp_pose(pos);
		return pos;
	}
#pragma endregion

}