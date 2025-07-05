/*****************************************************************
 * @文件名称:main.cpp
 * @创建人:dnp
 * @创建日期:2023-10-17
 * @简述: 结构体定义
 *
 * @更改历史:
 * 日期: 作者: 简述:
 *
*******************************************************************/
/**
 * @brief 解析后的指令
*/
#pragma once
#include <string>
#include <vector>

/**
 * @创建人:dnp
 * @创建日期:2023-10-18
 * @简述:操作模式没了类型(手动,自动,随动)
 * **/
enum OptMode { HAND_MODE = 1, PATH_MODE = 2, WYSIWYG_MODE=3 };

/**
 * @创建人:dnp
 * @创建日期:2023-10-18
 * @简述:机械臂状态
 * **/
enum EnumBotState : int {
	BS_NA = 0,         // 0:NA-未知,
	BS_NotReady = 1,   // 1:NotReady - 未就绪,
	BS_Ready = 2,      // 2:Ready-就绪,
	BS_Busy = 3,       // 3:Busy-执行中,
	BS_Pause = 4,      // 4:Pause-暂停,
	BS_ProtectiveStop = 5  // 5:ProtectiveStop-保护性停止！例如碰撞
};

/**
 * @创建人:dnp
 * @创建日期:2023-10-20
 * @简述:指令执行状态
 * **/
enum EnumCmdState :int {
	ECS_NotStart = 0,
	ECS_Running = 1,
	ECS_Terminated = 2
};

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
 * @创建日期:2023-10-26
 * @简述: 任务状态
 * **/
enum EnumTaskState {
	ST_Idle = 0, //任务未执行
	ST_Running = 1, //任务正在执行
	ST_Paused = 2, //任务已经暂停
	ST_Stopped = 3, //任务已经停止
	ST_Finished = 4, //任务已经正常执行完成，唯一表示任务正常	完成（任务已经结束）
	ST_Interrupt = 5, //任务被中断（任务已经结束）
	ST_Error = 6, //任务出错（任务已经结束）
	ST_Illegal = 7, //任务非法，当前状态下任务不能执行（任务	已经结束）
	ST_ParameterMismatch = 8 //任务参数错误（任务已经结束）
};

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

struct Cmd
{
	// 指令名称
	std::string name;

	// 指令参数(角度获取位姿,长度为6)
	std::vector<float> params;

	// 工具坐标系名称
	std::string tool = "";

	// 工件坐标系名称
	std::string workpiece = "";

	// 指令状态
	EnumCmdState state = EnumCmdState::ECS_NotStart;

	// 运行时间戳 (mm) 用于手动控制时候超时
	long runTs = 0;

	// 加速度
	double v = 0.1;

	// 加速度
	double a = 0.1;
};

/**
 * @创建人 dnp
 * @简介  随动控制指令
 */
struct WysiwygCmd
{
	// 指令作用的机器人
	int RobId=0;

	// 相机名称
	std::string cameraName;

	// 相机z方向增量
	double deltaX;

	// 相机y方向增量
	double deltaY;

	// 批次号 (忽略同一批次号的请求(查询除外))
	int batchNo;

	// 手眼异臂时候观测臂的关节角度
	std::vector<double> observeJoints;

	// 指令状态
	EnumCmdState state = EnumCmdState::ECS_NotStart;

	// 运行时间戳 (mm) 用于手动控制时候超时
	long runTs = 0;

	// 超时时间(毫秒)
	int timeoutMs = 0;

	// 接收到的批次号
	int receivedBatchNo = 0;

	// 是否是新的指令,如果是新的指令,但 state为运行时候,需要终止正在执行的指令
	bool terminatedNow = false;

	// 增量作为方向向量,目标位置不再是由deltax和delta有指定的点,而是有二者所指的方向.
	bool deltaAsDirection = false;

};

/**
 * @创建人:dnp
 * @创建日期:2023-10-17
 * @简述:指令集结构体
 * **/
struct Cmds {
	// 机器人ID
	int RobId;

	// 额外参数,根据上下文不同而异
	int param = 0;

	// 额外参数,根据上下文不同而异
	float paramf = 0.0;

	// 批次号 (忽略同一批次号的请求(查询除外))
	int batchNo;

	// 请求类型 getstate,path,hand,speed
	std::string reqType;

	// 指令集列表
	std::vector<Cmd> cmds;

	// 是否已经完成
	bool terminated = false;

	// 是否解析成功
	bool parseState = true;
};

/**
 * @创建人:dnp
 * @创建日期:2023-10-18
 * @简述:双臂状态和碰撞距离
 * **/
struct RobState
{
	// 机械臂1的状态
	EnumRobState rob1State = EnumRobState::SR_Start;

	// Nc程序运行状态
	EnumProgramState rob1ProgramState = EnumProgramState::SP_Stopped;

	// 安全状态
	EnumSafetyState rob1SafetyState = EnumSafetyState::SS_RUN;

	// 机器人关节
	std::vector<float> rob1Joints;

	// 末端坐标
	std::vector<float> rob1Tcps;

	// 机械臂2的状态
	EnumRobState rob2State = EnumRobState::SR_Start;

	// Nc程序运行状态
	EnumProgramState rob2ProgramState = EnumProgramState::SP_Stopped;

	// 安全状态
	EnumSafetyState rob2SafetyState = EnumSafetyState::SS_RUN;

	// 机器人关节
	std::vector<float> rob2Joints;

	// 末端坐标
	std::vector<float> rob2Tcps;

	// 机器人1错误代码
	int rob1ErrCode;

	// 机器人2 错误代码
	int rob2ErrCode;

	// 碰撞距离(单位mm,为0时候说明已经碰撞)
	int clashDistanceMm = -1;

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
 * @创建日期:2023-10-18
 * @简述:返回消息体
 * **/
struct Resp
{
	// 状态 0-失败  1-成功
	int status;

	// 消息内容
	std::string msg;
};