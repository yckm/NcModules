#include "DistanceHelper.h"
#include <fstream>
#include <iostream>
#include <chrono>
long GetTsMs() {
	auto now = std::chrono::high_resolution_clock::now();
	auto nano_time_point = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
	auto epoch = nano_time_point.time_since_epoch();
	return  std::chrono::duration_cast<std::chrono::milliseconds>(epoch).count();
}

void Clash::DistanceHelper::extract_nearest_points(std::vector<double>& nps, fcl::DistanceResultd& result)
{
	nps[0] = result.nearest_points[0][0];
	nps[1] = result.nearest_points[0][1];
	nps[2] = result.nearest_points[0][2];
	nps[3] = result.nearest_points[1][0];
	nps[4] = result.nearest_points[1][1];
	nps[5] = result.nearest_points[1][2];
}

std::tuple<double, std::vector<double>> Clash::DistanceHelper::calcDistance(std::vector<fcl::CollisionObject<double>*>& chain1, std::vector<fcl::CollisionObject<double>*>& chain2)
{
	std::vector<double> nps = { 0,0,0,0,0,0 };
	if (chain1.size() == 0 || chain2.size() == 0) {
		std::tuple<double, std::vector<double>> res = { -2,nps };
		return res;
	}

	fcl::DistanceRequest<double> request;
	request.enable_nearest_points = true;
	request.gjk_solver_type = fcl::GJKSolverType::GST_INDEP;
	fcl::DistanceResult<double> result;

	double minDist = 9999;

	int cnt = 0;
	
	for (int i = 0; i < chain1.size(); i++) {
		for (int j = 0; j < chain2.size(); j++) {
			result.clear();
			double dist = fcl::distance<double>(chain1[i], chain2[j], request, result);

			if (dist < 0) {
				extract_nearest_points(nps, result);
				std::tuple<double, std::vector<double>> res = { dist,nps };
				return res;
			}

			if (dist < minDist) {
				minDist = dist;
				extract_nearest_points(nps, result);
			}
		}
	}
	//printf("([%d,%d,%d),(%d,%d,%d)]\n", nps[0], nps[1], nps[2], nps[3], nps[4], nps[5]);
	std::tuple<double, std::vector<double>> res = {minDist,nps };
	return res;
}

void Clash::DistanceHelper::saveSampleFclModel(std::vector<std::string>& arr, std::string savePath, std::vector<double> nearest_points)
{
	arr.push_back(",\"nearest_points\":[" + std::to_string(nearest_points[0]) + "," + std::to_string(nearest_points[1]) + "," + std::to_string(nearest_points[2]) + "," + std::to_string(nearest_points[3]) + "," + std::to_string(nearest_points[4]) + "," + std::to_string(nearest_points[5]) + "]" );
	saveSampleFclModel(arr, savePath);
}

void Clash::DistanceHelper::saveSampleFclModel(std::vector<std::string>& arr, std::string savePath)
{	
	std::string fname = savePath+ std::to_string(GetTsMs()) + "_smaple.json";
	std::ofstream out(fname); // 打开输出文件
	if (out.is_open()) { // 判断文件是否打开成功
		out << "{";
		for (int i = 0; i < arr.size(); i++) {
			out << arr[i] ; // 将字符串数组的元素写入文件，每个元素占一行
		}
		out << "}";
		out.close(); // 关闭文件
	}

	std::cout << "\n--------model sample-----------------" << std::endl;
	std::cout << fname << std::endl;
	std::cout << "-------------------------------------\n" << std::endl;
}

std::vector<std::string > Clash::DistanceHelper::sampleFclModelToJson(std::vector<fcl::CollisionObject<double>*> fclCollisionObjs, int pointCnt)
{
	std::shared_ptr<fcl::CollisionGeometry<double>> g0 = std::make_shared<fcl::Sphere<double>>(1);
	fcl::CollisionObject<double>* ball = new fcl::CollisionObject<double>(g0);

	// 计算距离
	fcl::DistanceRequest<double> request;
	request.enable_nearest_points = true;
	request.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;
	fcl::DistanceResult<double> result;


	srand(time(NULL)); // 设置随机种子

	std::vector<std::string > arr;

	fcl::Vector3d cv(0, 0, 0);

	arr.push_back("\"pcds\":[");
	for (auto& fclCollisionObj : fclCollisionObjs) {
		arr.push_back("[");
		for (int i = 0; i < pointCnt; i++) {
			result.clear();
			cv[0] = rand() % 20001 - 10000;
			cv[1] = rand() % 20001 - 10000;
			cv[2] = rand() % 20001 - 10000;
			ball->setTranslation(cv);

			double dist = fcl::distance<double>(fclCollisionObj, ball, request, result);
			if (dist >= 0) {
				arr.push_back("[" + std::to_string(result.nearest_points[0][0]) + "," + std::to_string(result.nearest_points[0][1]) + "," + std::to_string(result.nearest_points[0][2]) + "],");
			}
		}
		arr.push_back("],");
	}

	arr.push_back("]");
	return arr;
}
