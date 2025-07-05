#pragma once
#include <string>
/*include lua 的头函数时候必须使用 extern "C"{} 包围住 */
extern "C" {
#include "lua/lua.h"
#include "lua/lualib.h"
#include "lua/lauxlib.h"
#include "lua/lua.hpp"
}
#include <stdio.h>
#include <iostream>
#include "string"
#include <vector>

namespace NcParse {
	/**
	 * @brief 解析后的指令
	*/
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

		// 是否合法(比如选择角度太大或者太小会导致机械臂运动失败等类似问题都判别为不合法)
		bool vaild = true;// 默认为合法

		float v = 0.1;

		float a = 0.1;
	};

	/**
	 * @brief 用户编写lua脚本回调函数定义
	*/
	class NcCallbackFunc
	{
	public:

		static std::vector<Cmd> cmds;
		/**
		 * @brief 清空解析好的指令集
		 * @return
		*/
		static void clear();

		/**
		 * @brief 获取解析好后的指令集
		 * @return 指令集数组
		*/
		static std::vector<Cmd> getCmds();

		/**
		* @brief 该指令控制机械臂从当前状态，按照关节运动的方式移动到目标关节角状态。
		* @param lua lua虚拟机
		* @return 返回参数个数
		*/
		static int moveJ(lua_State* lua);

		/**
		* @brief 该指令控制机械臂从当前状态，按照关节运动的方式移动到目标关节角状态。
		* @param lua lua虚拟机
		* @return 返回参数个数
		*/
		static int moveJ2(lua_State* lua);

		/**
		 * @brief 该指令控制机械臂从当前状态，按照关节运动的方式移动到末端目标位置。
		 * @param lua lua虚拟机
		 * @return 返回参数个数
		*/
		static int moveJpose(lua_State* lua);

		/**
		 * @brief 该指令控制机械臂末端从当前状态按照直线路径移动到目标状态。
		 * @param lua lua虚拟机
		 * @return 返回参数个数
		*/
		static int moveL(lua_State* lua);

		/**
		 * @brief 该指令控制机械臂沿工具坐标系直线移动一个增量。
		 * @param lua lua虚拟机
		 * @return 返回参数个数
		*/
		static int moveTcp(lua_State* lua);

		/**
		 * @brief 获取机器人状态。
		 * @param lua lua虚拟机
		 * @return 返回参数个数
		*/
		static int getRobState(lua_State* lua);

		/**
		 * @brief 睡眠n毫秒。
		 * @param lua lua虚拟机
		 * @return 返回参数个数
		*/
		static int sleep(lua_State* lua);
	};
}