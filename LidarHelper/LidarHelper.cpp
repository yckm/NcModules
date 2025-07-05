// LidarHelper.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include "LidarHelper.h"
#include "Utils/httplib.h"
#include <json.hpp>

using json = nlohmann::json;

/**
 * @创建人:dnp
 * @创建日期:2023-11-3
 * @简述:post返回消息体
 * **/
struct Msg {
	bool status = false;
	nlohmann::json_abi_v3_11_2::json data;
};

Msg Post(std::string host, std::string url, std::string body)
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
		auto res = cli.Post(url, headers, body, "application/json");
		if (res == nullptr || res->status != 200) {
			return msg;
		}

		msg.status = true;
		msg.data = json::parse(res->body);
		return msg;
	}
	catch (std::exception& e)
	{
		return msg;
	}
}

LidarHelper::LidarHelper(std::string p_host)
{
	host = p_host;
}

/**
 * @创建人:dnp
 * @简述:启动雷达
 * @返回值: 操作结果
 * **/
BasicMsg LidarHelper::start() {
	BasicMsg bm;

	auto resp = Post(this->host, "/lidar/start", "{}");
	if (!resp.status) {
		bm.msg = "操作失败";
		return bm;
	}

	bm.status = resp.data["status"];
	bm.msg = resp.data["msg"];

	return bm;
}

/**
 * @创建人:dnp
 * @简述:关闭雷达
 * @返回值:操作结果
 * **/
BasicMsg LidarHelper::close() {
	BasicMsg bm;

	auto resp = Post(this->host, "/lidar/close", "{}");
	if (!resp.status) {
		bm.msg = "操作失败";
		return bm;
	}

	bm.status = resp.data["status"];
	bm.msg = resp.data["msg"];

	return bm;
}

/**
* @创建人:dnp
* @简述:获取单帧点云
* @参数 isCombineFrames 是否获取合并后的帧( true 将多帧合并为一帧进行获取, false 获取一帧)
* @返回值:单帧点云
* **/
Pcd LidarHelper::getFrame(bool isCombineFrames) {
	Pcd pcd;

	std::string url = isCombineFrames ? "/lidar/combine" : "/lidar/single";

	auto resp = Post(this->host, url, "{}");
	if (!resp.status) {
		pcd.msg = "获取点云失败";
		return pcd;
	}

	int status = resp.data["status"];
	if (status == 0) {
		pcd.msg = resp.data["msg"];
		return pcd;
	}

	// 最近点
	for (auto& v : resp.data["data"]["lower"]) {
		pcd.lower.push_back(v);
	}

	// 点云
	for (auto& item : resp.data["data"]["pcd"]) {
		std::vector<float> pt;
		for (auto& v : item) {
			pt.push_back(v);
		}
		pcd.pcd.push_back(pt);
	}

	pcd.status = 1;
	return pcd;
}

/**
 * @创建人:dnp
 * @简述:获取电子围栏信息
 * @参数:distanceMm 相间距(单位 毫米)
 * @返回值: 边界信息
 * **/
Boundies LidarHelper::getBoundies(int distanceMm) {
	Boundies boundary;

	auto resp = Post(this->host, "/lidar/boundary", "{\"distance\":" + std::to_string(distanceMm) + ",\"cylinder\":0}");
	if (!resp.status) {
		boundary.msg = "获取点云失败";
		return boundary;
	}

	int status = resp.data["status"];
	if (status == 0) {
		boundary.msg = resp.data["msg"];
		return boundary;
	}

	// 围栏
	for (auto& item : resp.data["data"]["boundaries"]) {
		std::vector<float> pt;
		for (auto& v : item) {
			pt.push_back(v);
		}
		boundary.boundaries.push_back(pt);
	}

	// 环境
	for (auto& item : resp.data["data"]["env"]) {
		std::vector<float> pt;
		for (auto& v : item) {
			pt.push_back(v);
		}
		boundary.env.push_back(pt);
	}

	// obb顶点
	for (auto& item : resp.data["data"]["vertices"]) {
		std::vector<std::vector<float>> arr;
		for (auto& vertices : item) {
			std::vector<float> pt;
			for (auto& v : vertices)
			{
				pt.push_back(v);
			}
			arr.push_back(pt);
		}
		boundary.vertices.push_back(arr);
	}

	// 分段线缆obb x轴中线线的点云
	for (auto& item : resp.data["data"]["x_axis"]) {
		std::vector<std::vector<float>> arr;
		for (auto& vertices : item) {
			std::vector<float> pt;
			for (auto& v : vertices)
			{
				pt.push_back(v);
			}
			arr.push_back(pt);
		}
		boundary.Xaxis.push_back(arr);
	}

	boundary.status = 1;
	return boundary;
}

std::vector<BoundryCylinder> LidarHelper::getBoundryCylinder(int distanceMm)
{
	std::vector<BoundryCylinder> arr;

	auto resp = Post(this->host, "/lidar/boundary", "{\"distance\":" + std::to_string(distanceMm) + ",\"cylinder\":1}");
	if (!resp.status) {
		return arr;
	}

	int status = resp.data["status"];
	if (status == 0) {
		return arr;
	}

	for (auto& item : resp.data["data"]) {
		BoundryCylinder bc;
		bc.radius = item["radius"];
		bc.height = item["height"];
		bc.trans = item["trans"];

		arr.push_back(bc);
	}
	return arr;
}