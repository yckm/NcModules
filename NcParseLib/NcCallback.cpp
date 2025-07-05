#include "NcCallbackFunc.h"
namespace NcParse {
	std::vector<Cmd> NcCallbackFunc::cmds = {};
	/**
	 * @brief 清空解析好的指令集
	 * @return
	*/
	void NcCallbackFunc::clear() {
		std::vector <Cmd>().swap(NcCallbackFunc::cmds);  //清除容器并最小化它的容量
	}

	/**
	 * @brief 获取解析好后的指令集
	 * @return 指令集数组
	*/
	std::vector<Cmd> NcCallbackFunc::getCmds() {
		return NcCallbackFunc::cmds;
	}

	/**
	* @brief 该指令控制机械臂从当前状态，按照关节运动的方式移动到目标关节角状态。
	* @param lua lua虚拟机
	* @return 返回参数个数
	*/
	int NcCallbackFunc::moveJ(lua_State* lua) {
		int n = lua_gettop(lua);
		// std::cout << "stack num=>" << n << std::endl;
		if (n == 0) {
			lua_pushstring(lua, "fail");
			return 1; // 指明getRobState作为lua函数时返回值的个数
		}
		// ---------------------------------------------
		Cmd cmd;
		cmd.name = "moveJ";
		cmd.params.push_back(lua_tonumber(lua, 1)); // j1
		cmd.params.push_back(lua_tonumber(lua, 2)); // j2
		cmd.params.push_back(lua_tonumber(lua, 3)); // j3
		cmd.params.push_back(lua_tonumber(lua, 4)); // j4
		cmd.params.push_back(lua_tonumber(lua, 5)); // j5
		cmd.params.push_back(lua_tonumber(lua, 6)); // j6
		cmd.v=lua_tonumber(lua, 7);// v
		cmd.a = lua_tonumber(lua, 8);// a

		// 检查参数合法性 2023年10月26日
		for (float v : cmd.params) {
			if (v > 2 * 3.1416 || v < -2 * 3.1416) {
				cmd.vaild = false;//标记为不合法
				break;
			}
		}

		cmds.push_back(cmd);

		// 调用加入接口函数
		// ...
		std::string state = "callbackMoveJ.OK";
		lua_pushstring(lua, state.c_str());
		return 1; // 返回参数数量
	}
	/**
	* @brief 该指令控制机械臂从当前状态，按照关节运动的方式移动到目标关节角状态。
	* @param lua lua虚拟机
	* @return 返回参数个数
	*/
	int NcCallbackFunc::moveJ2(lua_State* lua) {
		int n = lua_gettop(lua);
		// std::cout << "stack num=>" << n << std::endl;
		if (n == 0) {
			lua_pushstring(lua, "fail");
			return 1; // 指明getRobState作为lua函数时返回值的个数
		}
		// ---------------------------------------------
		Cmd cmd;
		cmd.name = "moveJ2";
		cmd.params.push_back(lua_tonumber(lua, 1)); // j1
		cmd.params.push_back(lua_tonumber(lua, 2)); // j2
		cmd.params.push_back(lua_tonumber(lua, 3)); // j3
		cmd.params.push_back(lua_tonumber(lua, 4)); // j4
		cmd.params.push_back(lua_tonumber(lua, 5)); // j5
		cmd.params.push_back(lua_tonumber(lua, 6)); // j6
		cmd.v = lua_tonumber(lua, 7);// v
		cmd.a = lua_tonumber(lua, 8);// a

		// 检查参数合法性 2023年10月26日
		for (float v : cmd.params) {
			if (v > 2 * 3.1416 || v < -2 * 3.1416) {
				cmd.vaild = false;//标记为不合法
				break;
			}
		}

		cmds.push_back(cmd);

		// 调用加入接口函数
		// ...
		std::string state = "callbackMoveJ.OK";
		lua_pushstring(lua, state.c_str());
		return 1; // 返回参数数量
	}

	/**
	 * @brief 该指令控制机械臂从当前状态，按照关节运动的方式移动到末端目标位置。
	 * @param lua lua虚拟机
	 * @return 返回参数个数
	*/
	int NcCallbackFunc::moveJpose(lua_State* lua) {
		int n = lua_gettop(lua);
		// std::cout << "stack num=>" << n << std::endl;
		if (n == 0) {
			lua_pushstring(lua, "fail");
			return 1; // 指明getRobState作为lua函数时返回值的个数
		}

		// ---------------------------------------------
		Cmd cmd;
		cmd.name = "moveJpose";
		cmd.params.push_back(lua_tonumber(lua, 1)); // x
		cmd.params.push_back(lua_tonumber(lua, 2)); // y
		cmd.params.push_back(lua_tonumber(lua, 3)); // z
		cmd.params.push_back(lua_tonumber(lua, 4)); // rx
		cmd.params.push_back(lua_tonumber(lua, 5)); // ry
		cmd.params.push_back(lua_tonumber(lua, 6)); // rz
		cmd.v = lua_tonumber(lua, 7);// v
		cmd.a = lua_tonumber(lua, 8);// a
		cmd.tool = lua_tostring(lua, 9);// tool
		cmd.workpiece = lua_tostring(lua, 10);// 工件坐标
		cmds.push_back(cmd);

		// 调用加入接口函数
		// ...
		std::string state = "callbackMoveJpose.OK";
		lua_pushstring(lua, state.c_str());
		return 1; // 返回参数数量
	}

	/**
	 * @brief 该指令控制机械臂末端从当前状态按照直线路径移动到目标状态。
	 * @param lua lua虚拟机
	 * @return 返回参数个数
	*/
	int NcCallbackFunc::moveL(lua_State* lua) {
		int n = lua_gettop(lua);
		// std::cout << "stack num=>" << n << std::endl;
		if (n == 0) {
			lua_pushstring(lua, "fail");
			return 1; // 指明getRobState作为lua函数时返回值的个数
		}

		// ---------------------------------------------
		Cmd cmd;
		cmd.name = "moveL";
		cmd.params.push_back(lua_tonumber(lua, 1)); // x
		cmd.params.push_back(lua_tonumber(lua, 2)); // y
		cmd.params.push_back(lua_tonumber(lua, 3)); // z
		cmd.params.push_back(lua_tonumber(lua, 4)); // rx
		cmd.params.push_back(lua_tonumber(lua, 5)); // ry
		cmd.params.push_back(lua_tonumber(lua, 6)); // rz
		cmd.v = lua_tonumber(lua, 7);// v
		cmd.a = lua_tonumber(lua, 8);// a		
		cmd.tool = lua_tostring(lua, 9);// tool
		cmd.workpiece = lua_tostring(lua, 10);// 工件坐标
		cmds.push_back(cmd);

		// 调用加入接口函数
		// ...
		std::string state = "callbackMoveL.OK";
		lua_pushstring(lua, state.c_str());
		return 1; // 返回参数数量
	}

	/**
	 * @brief 该指令控制机械臂沿工具坐标系直线移动一个增量。
	 * @param lua lua虚拟机
	 * @return 返回参数个数
	*/
	int NcCallbackFunc::moveTcp(lua_State* lua) {
		int n = lua_gettop(lua);
		// std::cout << "stack num=>" << n << std::endl;
		if (n == 0) {
			lua_pushstring(lua, "fail");
			return 1; // 指明getRobState作为lua函数时返回值的个数
		}

		// ---------------------------------------------
		Cmd cmd;
		cmd.name = "moveTcp";
		cmd.params.push_back(lua_tonumber(lua, 1)); // x
		cmd.params.push_back(lua_tonumber(lua, 2)); // y
		cmd.params.push_back(lua_tonumber(lua, 3)); // z
		cmd.params.push_back(0); // rx
		cmd.params.push_back(0); // ry
		cmd.params.push_back(0); // rz
		cmd.v = lua_tonumber(lua, 4);// v
		cmd.a = lua_tonumber(lua, 5);// a
		cmd.tool = lua_tostring(lua, 6);// tool
		cmds.push_back(cmd);

		// 调用加入接口函数
		// ...
		std::string state = "callbackTcpMove.OK";
		lua_pushstring(lua, state.c_str());
		return 1; // 返回参数数量
	}

	/**
	 * @brief 获取机器人状态。
	 * @param lua lua虚拟机
	 * @return 返回参数个数
	*/
	int NcCallbackFunc::getRobState(lua_State* lua) {
		// std::cout << "stack num=>" << lua_gettop(lua) << std::endl;
	   // 调用加入接口函数
	   // ...
		int state = 0;
		lua_pushnumber(lua, state);
		return 1; // 返回参数数量
	}

	/**
	   * @brief 睡眠n秒。
	   * @param lua lua虚拟机
	   * @return 返回参数个数
   */
	int NcCallbackFunc::sleep(lua_State* lua) {
		int n = lua_gettop(lua);
		// std::cout << "stack num=>" << lua_gettop(lua) << std::endl;
		if (n == 0) {
			lua_pushstring(lua, "fail");
			return 1; // 指明getRobState作为lua函数时返回值的个数
		}

		// ---------------------------------------------
		Cmd cmd;
		cmd.name = "sleep";
		cmd.params.push_back(lua_tonumber(lua, 1)); // 毫秒
		cmds.push_back(cmd);

		// 调用加入接口函数
		// ...
		std::string state = "sleep.OK";
		lua_pushstring(lua, state.c_str());
		return 0; // 返回参数数量
	}
}