#pragma once
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
#include "NcCallbackFunc.h"

#define NCPARSEAPI __declspec(dllexport)

namespace NcParse {
    class NCPARSEAPI Parser
    {
    public:
        /**
         * @brief 将用户脚本解释为Cmd数组
         * @param scriptContent 用户编写脚本内容
         * @return Cmd数组
        */
        static std::vector<Cmd> Parse(std::string scriptContent) {
            NcCallbackFunc::clear();  //清除容器并最小化它的容量

            lua_State* lua = luaL_newstate();
            try {
                luaL_openlibs(lua);// 载入Lua基本库 

                // 注册回调函数
                lua_register(lua, "moveJ", NcCallbackFunc::moveJ);
                lua_register(lua, "moveJ2", NcCallbackFunc::moveJ2);
                lua_register(lua, "moveJpose", NcCallbackFunc::moveJpose);
                lua_register(lua, "moveL", NcCallbackFunc::moveL);
                lua_register(lua, "moveTcp", NcCallbackFunc::moveTcp);
                lua_register(lua, "getRobState", NcCallbackFunc::getRobState);
                lua_register(lua, "sleep", NcCallbackFunc::sleep);


                // 执行lua脚本
                // std::cout << luaL_loadfile(lua, "user.lua") << std::endl;


                std::cout << scriptContent << std::endl;

                std::cout << luaL_loadstring(lua, scriptContent.c_str()) << std::endl;

                std::cout << lua_pcall(lua, 0, 0, 0) << std::endl;  // 必须先执行,否则取不到参数

                /* 调用echo函数
                std::cout << "get echo "<< lua_getglobal(lua, "echo") << std::endl;
                lua_pushstring(lua,"from c++");
                std::cout << lua_pcall(lua, 1, 1, 0) << std::endl;
                */
            }
            catch (const char*& e) {
                std::cout << "解析NC脚本失败: " << e << std::endl;
                lua_close(lua);// 释放 Lua 状态机
                std::vector<Cmd> fail = {};
                return fail;
            }

            lua_close(lua);// 释放 Lua 状态机
            return NcCallbackFunc::getCmds();
        }
    };
}