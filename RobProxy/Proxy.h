#pragma once
#include <json.hpp>
#include "Models.h"
#include <shared_mutex>
using json = nlohmann::json;

namespace mainthread {
	class Proxy
	{
	public:
#pragma region  常量
		// 获取机械臂状态
		static const std::string reqTypeGetState;

		// 执行规划路径
		static const std::string reqTypePath;

		// 执行手动控制
		static const std::string reqTypeHand;

		// 手动自动切换
		static const std::string reqTypeSwitch;

		// 速度调节
		static const std::string reqTypeSpeedAdjust;

		// 清空所有指令并将机器设为就绪状态
		static const std::string reqTypeClearAllAndStandby;

		// 关节指令
		static const std::string reqTypeShutdown;

		// 机械臂1 运动速率
		static int globalSpeedRate1;

		// 机械臂2 运动速率
		static int globalSpeedRate2;

		// 当前批次的执行状态
		static int CURBATCHNO;
		static int CURBATCHTERMINATED;// 0 未终结 1 已终结

		// 随动指令
		static const std::string reqTypeWysiwyg;

#pragma endregion

#pragma region 锁

#pragma endregion

#pragma region 变量
		static OptMode mode; // 是否手动控制
		static Cmds* cmds; // 从客户端接收的指令集
		static WysiwygCmd* wysiwygCmd; //随动控制指令
		static RobState* robState; // 机器状态
		static int arm2ArmDistance;// 机械臂之间的距离(单位mm)
		static int arm2EnvDistanceMm;//  机械臂到环境的距离(mm)
#pragma endregion

#pragma region MyRegion
		/**
		 * @创建人:dnp
		 * @简述:解析用户请求参数
		 * @参数:body 用户请求消息体
		 * **/
		static Resp ParseUserRequestAndDistribute(const Cmds* usrCmds);

		/**
		 * @创建人 dnp
		 * @简介 机械化和分配随动控制指令
		 * @参数 moveRobId 需要移动的机械臂
		 * @参数 s 随动指令 ( sd:cameraName:x:y:timeoutMs )
		 * @参数 batchNo 批次号
		 * @返回值 是否成功
		 */
		static Resp ParseAndDistributeWysiwyg(int moveRobId, std::string s,int batchNo);

		/**
		 * @创建人:dnp
		 * @简述:更新机器人状态
		 * @参数: robId 机器人ID
		 * @参数: robProgramState Nc运行状态
		 * @参数: robState 机器人状态
		 * @参数: joints 关节角度
		 * @参数: tcps tcps位姿
		 * @参数: safetyState 安全状态
		 * @参数: clashSign 碰撞信号
		 * @参数: clashArm 碰撞臂
		 * **/
		static void UpdateRobState(int robId, EnumRobState p_robState, EnumProgramState robProgramState, std::vector<float> joints, std::vector<float> tcps, EnumSafetyState safetyState, int errCode, int clashSign, int clashArm);

		/**
		 * @创建人:dnp
		 * @创建日期:2023-10-18
		 * @简述: 获取双臂状态
		 * **/
		static std::string  GetRobState();

		/**
		 * @创建人:dnp
		 * @创建日期:2023-10-18
		 * @简述:更新子线程的指令集
		 * @参数: subCmd 子线程指令集
		 * @参数: sdCmd 子线程随动指令
		 * @返回值: <接收当前指令时候的操作模式,机器人ID,机械臂全局速率>
		 * **/
		static std::tuple< OptMode, int, int> UpdateSubCmds(Cmds* subCmd,WysiwygCmd* sdCmd);

		/**
		 * @创建人:dnp
		 * @简述:更新碰撞距离
		 * @参数:arm2armDistance 机械臂之间碰撞距离(单位mm)
		 * @参数: arm2envDistance 机械臂都环境的碰撞距离
		 * **/
		static void UpdateClashDistance(int arm2armDistance, int arm2envDistance);

		/**
		 * @创建人:dnp
		 * @简述:预测是否将要碰撞(根据距离和距离变化进行预测)
		 * @返回 ture 将要碰撞  false 不会发送碰撞
		 * **/
		static bool WillClash();

		/**
		 * @创建人:dnp
		 * @创建日期:2023-11-9
		 * @简述:获取碰撞距离
		 * @返回值: (臂到臂的距离,臂到环境的距离)
		 * **/
		static std::tuple<int, int> GetClashDistance();

		/**
		 * @创建人:dnp
		 * @简述:机器人是否处于可用状态
		 * @参数:robId 机械臂ID
		 * @返回值:机械臂状态
		 * **/
		static std::tuple<EnumRobState, EnumSafetyState> GetRobState(int robId);

		/**
		 * @创建人:dnp
		 * @创建日期:2023-11-9
		 * @简述: 获取机械臂关节角度
		 * @返回值: [1关节角度,2关节角度]
		 * **/
		static std::vector<std::vector<float>> GetJoints();
#pragma endregion

#pragma region batchNo
		/**
		 * @创建人:dnp
		 * @简述:设置指定批次号的指令集未结束状态
		 * @参数:batchNo 批次号
		 * **/
		static void UpdateBatchTerminated(int batchNo);
#pragma endregion

		private:
			/**
			 * @创建人 dnp
			 * @简介 拷贝随动指令
			 * @参数 sdCmd
			 */
			static void CopyWysiwygCmd(WysiwygCmd* sdCmd);
	};
}