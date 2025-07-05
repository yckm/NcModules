#pragma once
#include <string>
#include <vector>
#include <json.hpp>
#include "Utils/base64/base64.h"
#include "include/Models.h"

using json = nlohmann::json;

namespace RobHelper {
	std::string LuaModel::toJson() {
		json body;
		body["RobId"] = robId;
		body["batchNo"] = batchNo;
		body["type"] = type;
		body["param"] = param;
		body["paramf"] = paramf;
		body["style"] = "lua";
		body["params"] = base64_encode(luaContent, false);
		return body.dump();
	}

	/**
	 * @创建人:dnp
	 * @简述:转换为json字符串
	 * @返回值:json字符串
	 * **/
	std::string JsonModel::toJson() {
		json body;
		body["RobId"] = robId;
		body["batchNo"] = batchNo;
		body["type"] = type;
		body["param"] = param;
		body["paramf"] = paramf;
		body["style"] = "json";

		std::vector<nlohmann::json_abi_v3_11_2::json> cmds;
		for (const Cmd& c : params) {
			json cmd;
			cmd["name"] = c.name;
			cmd["tool"] = c.tool;
			cmd["workpiece"] = c.workpiece;
			cmd["params"] = c.params;
			cmd["v"] = c.v;
			cmd["a"] = c.a;
			cmds.push_back(cmd);
		}
		body["params"] = cmds;

		return body.dump();
	}
}