#pragma once
#include <iostream>
#include "../NcParseLib/Parser.h"
#include "Models.h"
#include <json.hpp>
#include "Utils/base64/base64.h"
using json = nlohmann::json;

namespace mainthread {
	class ScriptParser
	{
	private:
		Cmds* cmds = nullptr;

	public:
		ScriptParser() {
			cmds = new Cmds();
		}
		~ScriptParser() {
			if (cmds != nullptr) {
				delete cmds;
			}
		}
		/**
昆明东电销售合同20230424-16.pdf

		 * @创建人:dnp
		 * @简述:将用户请求的json的指令转换为cmds
		 * @参数:body 用户请求json
		 * **/
		Cmds* Parse(nlohmann::json_abi_v3_11_2::json& body) {
			std::string style = body["style"];// lua or json
			cmds->RobId = body["RobId"];
			cmds->batchNo = body["batchNo"];
			cmds->reqType = body["type"];
			cmds->param = body["param"];
			cmds->paramf = body["paramf"];

			if (style == "lua") { // 继续lua
				std::string lua = DecoceLua(body["params"]);
				std::vector< NcParse::Cmd>  rawCmds = NcParse::Parser::Parse(lua);

				// 一条指令都没有说明解析失败或者不合法了
				if (rawCmds.empty()) {
					cmds->parseState = false;
					return cmds;
				}

				for (NcParse::Cmd c : rawCmds) {
					if (!c.vaild) { // 含有不合法的指令或数值
						cmds->parseState = false;
						return cmds;
					}

					Cmd cmd;
					cmd.name = c.name;
					cmd.params = c.params;
					cmd.tool = c.tool;
					cmd.workpiece = c.workpiece;
					cmd.v = c.v;
					cmd.a = c.a;
					cmds->cmds.push_back(cmd);
				}
			}
			else { // 解析json
				for (auto& element : body["params"]) {
					Cmd cmd;
					cmd.name = element["name"];
					cmd.tool = element["tool"];
					cmd.workpiece = element["workpiece"];
					cmd.state = EnumCmdState::ECS_NotStart;
					cmd.v = element["v"];
					cmd.a = element["a"];
					for (auto& p : element["params"]) {
						cmd.params.push_back(p);
					}
					cmds->cmds.push_back(cmd);
				}
			}

			return cmds;
		}

		/**
		 * @创建人:dnp
		 * @简述:将bsse64解析为普通字符串
		 * @参数:base64luaContent
		 * @返回值:lua脚本内容
		 * **/
		static std::string DecoceLua(std::string base64luaContent) {
			if (base64luaContent.empty()) {
				return "";
			}
			return  base64_decode(base64luaContent);
		}
	};
}