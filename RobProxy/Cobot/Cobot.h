#pragma once
#include "DucoCobot.h"
#include <shared_mutex>

// #define COBOTAPI __declspec(dllexport)

namespace Cobot {
	class Cobot
	{
	public:
		/// <summary>
		/// 构造函数
		/// </summary>
		/// <param name="p_ip">机器人IP</param>
		/// <param name="p_port">机器人端口</param>
		/// <param name="p_channel">机器人通道</param>
		Cobot(std::string p_ip, int p_port, int p_channel);
		~Cobot();

		/// <summary>
		/// 连接机器人
		/// </summary>
		bool connect();

		/// 获取任务Id
		int getTaskState(int taskId);

#pragma region 系统控制 (默认阻塞)
		/**
		 * @创建人 dnp
		 * @简介 断开实例和NC的连接
		 * @返回值 
		 */
		int close();

		/// <summary>
		/// 上电
		/// </summary>
		/// <param name="block">是否阻塞(默认阻塞)</param>
		/// <returns></returns>
		int powerOn(bool block = true);

		/// <summary>
		/// 机器人上使能
		/// </summary>
		/// <param name="block">是否阻塞(默认阻塞)</param>
		/// <returns></returns>
		int enable(bool block = true);

		/// <summary>
		/// 机器人下电
		/// </summary>
		/// <param name="block">是否阻塞(默认阻塞)</param>
		/// <returns></returns>
		int powerOff(bool block = true);

		/// <summary>
		/// 机器人下使能
		/// </summary>
		/// <param name="block">是否阻塞(默认阻塞)</param>
		/// <returns></returns>
		int disable(bool block = true);

		/// <summary>
		/// 让机器人处于就绪状态
		/// </summary>
		/// <param name="block">是否阻塞(默认阻塞)</param>
		/// <returns></returns>
		int standby();

		/// <summary>
		/// 关闭机器人
		/// </summary>
		/// <returns></returns>
		int shutdown();
#pragma endregion

#pragma region 任务控制
		/// <summary>
		/// 停止所有任务。
		/// </summary>
		/// <param name="block">是否阻塞(默认阻塞)</param>
		/// <returns></returns>
		int stop(bool block = true);
		/// <summary>
		/// 暂停所有任务。
		/// </summary>
		/// <param name="block">是否阻塞(默认阻塞)</param>
		/// <returns></returns>
		int pause(bool block = true);
		/// <summary>
		/// 恢复所有暂停的任务。
		/// </summary>
		/// <param name="block">是否阻塞(默认阻塞)</param>
		/// <returns></returns>
		int resume(bool block = true);

		/**
		 * @创建人 dnp
		 * @简介 重置碰撞检测警告
		 */
		int collision_detection_reset();


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
		int moveJ(std::vector<double> jointLs, double v = 20, double a = 20, bool block = true);

		/// <summary>
		/// 该指令控制机械臂从当前状态，按照关节运动的方式移动到目标关节角状态。
		/// </summary>
		/// <param name="jointLs">数组对应1-6 关节的目标关节角度，单位rad</param>
		/// <param name="v">关节角速度，单位（rad/s），取值范围（0,100]</param>
		/// <param name="a">关节角加速度，单位（rad/s2），取值范围（0, 100]</param>
		/// <param name="block">指令是否阻塞型指令，如果为false 表示非阻塞指令，指令会立即返回</param>
		/// <returns>当配置为阻塞执行，返回值代表当前任务结束时的状态，若无融合为Finished, 若有融合为Interrupt。当配置为非阻塞执行，返回值代表当前任务的id。</returns>
		int moveJ2(std::vector<double> jointLs, double v, double a, bool block);

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
		int movePose(std::vector<double> pose, double v, double a, std::string tool, std::string wobj, bool block = false);

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
		int moveL(std::vector<double> pose, double v, double a, std::string tool, std::string wobj, bool block = false);

		/// <summary>
		/// 该指令控制机械臂沿工具坐标系直线移动一个"增量"。
		/// </summary>
		/// <param name="offset_in_tool">表示工具坐标系下的位姿偏移量。</param>
		/// <param name="v">直线移动的速度，单位（m/s），当x、y、z 均为0 时，线速度按比例换算成角速度。</param>
		/// <param name="a">加速度，单位（m/ s2）</param>
		/// <param name="tool">设置使用的工具的名称，为空时默认为当前使用的工具。</param>
		/// <param name="block">指令是否阻塞型指令，如果为false 表示非阻塞指令，指令会立即返回。</param>
		/// <returns>当配置为阻塞执行，返回值代表当前任务结束时的状态，若无融合为Finished, 若有融合为Interrupt。当配置为非阻塞执行，返回值代表当前任务的id。</returns>
		int tcpMove(std::vector<double> offset_in_tool, double v, double a, std::string tool, bool block = false);

		/**
		 * @brief 样条运动函数,该指令控制机器人按照空间样条进行运动。
		 * @param plist 在设置工件坐标系下的末端位姿列表,最多不超过50 个点，格式如下：[p1,p2,p3,…,pi,…]其中pi 为空间位姿，如[0.4,0.5,0.5,1.2,0.717,1.414]。
		 * @param v 末端速度，单位（m/s）。
		 * @param a 末端加速度，单位（m/s2）。
		 * @param tool 设置使用的工具的名称，为空时默认为当前使用的工具。
		 * @param wobj 设置使用的工件坐标系的名称，为空默认为当前使用的工件坐标系
		 * @param block block : 指令是否阻塞型指令，如果为false 表示非阻塞指令，指令会立即返回。
		*/
		int spline(std::vector<std::vector<double>> plist, double v, double a, std::string tool, std::string wobj, bool block = true);

#pragma endregion

#pragma region 实时控制
		/**
		 * @创建人:dnp
		 * @简述:该指令控制机械臂每个关节按照给定的速度一直运动，函数执行后会直接运行后续指令。运行speedj 函数后，机械臂会持续运动并忽略后续运动指令，直到接收到speed_stop()函数后停止。
		 * @参数:joints_v_list 每个关节的速度，单位rad/s。
		 * @参数:a  主导轴的关节加速度，单位rad/ s2
		 * @参数:time 运行时间，到达时间后会停止运动,单位ms。默认-1 表示一直运行(默认设置为200ms)。
		 * @参数:block 指令是否阻塞型指令，如果为false 表示非阻塞指令，指令会立即返回。
		 * @返回值:当配置为阻塞执行，返回值代表当前任务结束时的状态，当配置为非阻塞执行， 返回值代表当前任务的id 信息， 用户可以调用get_noneblock_taskstate（id）函数查询当前任务的执行状态。
		 * **/
		int speedJ(std::vector<double>  joints_v_list, double  a, int  time = 200, bool block = false);

		/**
		 * @创建人:dnp
		 * @简述:该指令控制机械臂末端按照给定的速度一直运动，函数执行后会直接运行后续指令。运行speedl 函数后，机械臂会持续运动并忽略后续运动指令，直到接收到speed_stop()函数后停止。
		 * @参数:tcp_v_List 末端速度向量，线速度单位m/s,角速度单位rad/s。
		 * @参数:a 末端的线性加速度，单位rad/ s2
		 * @参数:time 运行时间，到达时间会停止运动，单位ms。默认-1表示一直运行(默认设置为200ms)
		 * @参数:block 指令是否阻塞型指令，如果为false 表示非阻塞指令，指令会立即返回
		 * @返回值: 当配置为阻塞执行，返回值代表当前任务结束时的状态，当配置为非阻塞执行， 返回值代表当前任务的id 信息， 用户可以调用get_noneblock_taskstate（id）函数查询当前任务的执行状态。
		 * **/
		int speedL(std::vector<double> tcp_v_List, double a, int time = 200, bool block = false);

		/**
		 * @创建人:dnp
		 * @简述:停止speed运动
		 * @参数 block 是否为阻塞指令
		 * @返回值:当配置为阻塞执行，返回值代表当前任务结束时的状态，当配置为非阻塞执行， 返回值代表当前任务的id 信息， 用户可以调用get_noneblock_taskstate（id）函数查询当前任务的执行状态。
		 * **/
		int speedStop(bool block = true);

		/**
		 * @创建人:dnp
		 * @简述:设置机器人全局速率
		 * @参数:val 全局速率
		 * @返回值: 当前任务执行状态
		 * **/
		int speed(double val);
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
		int spline(double** plist, int pointNum, double v, double a, char* tool, char* wobj, bool block);
		
		/**
		 * @创建人 dnp
		 * @简介 该函数将一组points 点位信息输入到机器人控制器中的轨迹池。最大数量为1000 个
		 * @参数 joints 关节角度列表
		 * @参数 pointNum 关节角度组数
		 * @参数 block 是否阻塞
		 * @返回值 轨迹池当前存量
		 */
		int  trackEnqueue(double** joints, int pointNum, bool clearQueue, bool block);
		
		/**
		 * @创建人 dnp
		 * @简介 该函数用于将机器人控制器中的轨迹池清空。
		 * @返回值 阻塞执行，返回值代表当前任务结束时的状态。
		 */
		int  trackClear();

		/**
		 * @创建人 dnp
		 * @简介 该函数用于获取机器人控制器中的当前轨迹池大小。
		 * @返回值  阻塞执行，返回值为当前轨迹池大小。
		 */
		int  trackSize();

		/**
		 * @创建人 dnp
		 * @简介 该函数执行时，机器人的各关节将顺序到达轨迹池中的点位值直到轨迹池中无新的点位。执行过程中，主导关节（关节位置变化最大的关节）将以speed 与acc 规划运动，其他关节按比例缩放。注：如果已经开始执行停止规划，将不再重新获取轨迹池中的数据，直到完成停止。停止后如果轨迹池中有新的点位，将重新执行跟随。为保证运动连续性，建议至少保证轨迹池中有10 个数据。
		 * @参数 v 最大关节速度，单位(rad/s）。
		 * @参数 a 最大关节加速度，单位(rad/s2）。
		 * @参数 block 指令是否阻塞型指令，如果为false表示非阻塞指令，指令会立即返回。
		 * @返回值
		 */
		int  trackMove(double v, double a, bool block);
		
		/**
		 * @创建人 dnp
		 * @简介 将一组js连续执行
		 * @参数 joints 关节角度列表
		 * @参数 v 速度 (rad/s)
		 * @参数 a 加速度(rad/s2)
		 * @参数 block 是否阻塞
		 * @返回值 轨迹池当前存量
		 */
		int moveJs(std::vector<std::vector<double>> joints, double v, double a, bool block);
#pragma endregion

#pragma region 获取数据
		/**
		 * @创建人 dnp
		 * @简介 获取关节角度
		 * @返回值 [j1,j2,j3,j4,j5,j6]
		 */
		std::vector<double> getJoints();

		/**
		 * @创建人 dnp
		 * @简介 获取当前tcp位姿
		 * @返回值 [x,y,z,rx,ry,rz]
		 */
		std::vector<double> getTcpPose();
#pragma endregion



	private:
		/// <summary>
		/// 机器人IP
		/// </summary>
		std::string ip;

		/// <summary>
		/// 机器人端口
		/// </summary>
		int port = 7003;

		/// <summary>
		/// 机器人实例
		/// </summary>
		DucoRPC::DucoCobot* bot = nullptr;// 2.9版本以前不添加DucoRPC

		/// <summary>
		/// 机器人通道
		/// </summary>
		int channel = 0;
	};
}