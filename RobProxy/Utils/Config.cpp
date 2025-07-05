#include "Config.h"
Utils::ConfigDict* Utils::Config::dic = new ConfigDict();



Utils::Camera Utils::Config::getCamera(std::string key) {
	return dic->get(key);
}

std::vector<double> Utils::Config::get1Arr(std::string key) {
	return dic->get1Arr(key);
}


void Utils::Config::init() {
	std::ifstream f("configs.conf");
	std::stringstream buffer;
	buffer << f.rdbuf();
	auto body = nlohmann::json::parse(buffer.str());
	for (const auto& v : body.items()) {
		auto& data = v.value();
		std::string type = data["type"];
		int robId = data["armId"];

		// 变换矩阵
		if (type == "4x4") {
			std::vector<std::vector<double>> temp = data["value"];
			dic->update(v.key(),robId, temp);
			continue;
		}

		// 一维数组
		if (type == "vector<double>") {
			std::vector<double> vs=data["value"];
			dic->update(v.key(), vs);
		}
		
	}
	LOG(INFO) << "配置加载完成";
}
