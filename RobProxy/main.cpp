/*****************************************************************
 * @文件名称:main.cpp
 * @创建人:dnp
 * @创建日期:2023-10-17
 * @简述:函数入口
 *
 * @更改历史:
 * 日期: 作者: 简述:
 *
*******************************************************************/
#include <iostream>
#include "Utils/httplib.h"
#include <json.hpp>
#include "Models.h"
#include <shared_mutex>
#include "Proxy.h"
#include "Controller/NcControl.h"
#include "Cobot/StateRecv.h"
#include "ScriptParser.h"
#include "Utils/base64/base64.h"
#include "glog/logging.h"
#include "Clash/DistanceDectector.h"
#include "Utils/Config.h";
#include "Cobot/Cobot.h"
#include "Wysiwyg/Wysiwyg.h"

using json = nlohmann::json;

static void InitLogger() {
	google::InitGoogleLogging("robproxylog");

	// 设置特定严重级别的日志的输出目录和前缀
	google::SetLogDestination(google::GLOG_INFO, "C:\\ddtch\\svrlogs\\RobProxy_Info_");
	//google::SetLogDestination(google::GLOG_WARNING, "C:\\ddtch\\svrlogs\\RobProxy_Warn_");
	google::SetLogDestination(google::GLOG_ERROR, "C:\\ddtch\\svrlogs\\RobProxy_Err_");
	//google::SetLogDestination(google::GLOG_FATAL, "C:\\ddtch\\svrlogs\\RobProxy_Fatal_");

	//在日志文件名中级别后添加一个扩展名。适用于所有严重级别
	google::SetLogFilenameExtension(".log");

	//大于指定级别的日志都输出到标准输出
	google::SetStderrLogging(google::GLOG_INFO);

	// 设置写入文件日志水平
	FLAGS_v = 1;

	//实时输出日志
	FLAGS_logbufsecs = 0;

	//最大日志大小（MB）
	FLAGS_max_log_size = 10;

	//设置输出到屏幕的日志显示相应颜色
	FLAGS_colorlogtostderr = true;
}
/**
 * @创建人:dnp
 * @简述:返回数据库给客户端
 * @参数:res http相应
 * @参数:status 状态
 * @参数:msg 提示
 * @参数:data 消息体
 * **/
void ToResponse(httplib::Response& res, int status, std::string msg, std::string data)
{
	json result{
		{ "status", status },
		{ "msg", msg },
		{ "data", data }
	};
	res.set_content(result.dump(), "text/plain");
}

/**
 * @创建人:dnp
 * @简述: 运行再主线程中http服务,用来接收用户请求和返回数据给用户
 * **/
void HttpServer(int port = 8000) {
	httplib::Server svr;

	// 执行指令
	svr.Post("/nc/proxy", [](const httplib::Request& req, httplib::Response& res) {
		auto body = json::parse(req.body);
		mainthread::ScriptParser parser;
		Cmds* cmds = parser.Parse(body);
		if (cmds == nullptr || !cmds->parseState) {
			ToResponse(res, 0, "Script contains error", "");
			return;
		}

		if (cmds->reqType != "Hand") {
			LOG(INFO) << req.body;
		}

		// 更新环境模式模型
		if (cmds->reqType == "PCD") {
			int safeDistance = cmds->param;
			auto rp = Clash::DistanceDectector::upateEnv(safeDistance);
			ToResponse(res, rp.status, rp.msg, "");
			return;
		}

		Resp resp = mainthread::Proxy::ParseUserRequestAndDistribute(cmds);
		Clash::DistanceDectector::activateArm(cmds->RobId); // 激活需要检测碰撞的机械臂 2023年11月9日
		ToResponse(res, resp.status, resp.msg, "");
		});

	// 获取机器人状态
	svr.Get("/nc/proxy", [](const httplib::Request& req, httplib::Response& res) {
		std::string state = mainthread::Proxy::GetRobState();
		ToResponse(res, 1, "Success", state);
		});

	svr.listen("0.0.0.0", port);
}

#pragma endregion

int main()
{	
	InitLogger();
	Utils::Config::init();
	

	std::vector<double> wysiwygRange = Utils::Config::get1Arr("WyswygRange");

	// --------------机械臂1----------------------------------------------------------------
	int rob1Id = 1; // 机器人ID
	std::string ip1 = "192.168.6.202"; //机器人ip
	std::string rob1RecvPort = "2001";//机器人推送接收端口
	int  rob1port = 7003; // 机器人控制端口

	Cobot::StateRecv* stateRecv1 = new Cobot::StateRecv(rob1Id, ip1, rob1RecvPort);
	std::thread nc1Recv(&Cobot::StateRecv::ReceiveFromServer, stateRecv1);
	Controller::NcControl* nc1 = new Controller::NcControl(rob1Id, ip1, rob1port, wysiwygRange[0], wysiwygRange[1]); // 主机械臂
	std::thread nc1Control(&Controller::NcControl::Run, nc1);
	LOG(INFO) << "启动机械臂1服务";
	
	// --------------机械臂2----------------------------------------------------------------
	int rob2Id = 2;// 机器人ID
	std::string ip2 = "192.168.6.201";//机器人ip
	std::string rob2RecvPort = "2001";//机器人推送接收端口
	int  rob2port = 7003;// 机器人控制端口

	Cobot::StateRecv* stateRecv2 = new Cobot::StateRecv(rob2Id, ip2, rob2RecvPort);
	std::thread nc2Recv(&Cobot::StateRecv::ReceiveFromServer, stateRecv2);
	Controller::NcControl* nc2 = new Controller::NcControl(rob2Id, ip2, rob2port, wysiwygRange[0], wysiwygRange[1]); // 主机械臂
	std::thread nc2Control(&Controller::NcControl::Run, nc2);
	LOG(INFO) << "启动机械臂2服务";

	// -------------碰撞检测----------------------------------------------------------------
	std::string pcdServerHost = "http://127.0.0.1:8000";
	Clash::DistanceDectector::setMain2AssistMat(Utils::Config::getCamera("Main2AssistMat").mat);
	std::thread clashCalc(Clash::DistanceDectector::Run);
	
	// -------------http 服务----------------------------------------------------------------	
	HttpServer(8001);
}