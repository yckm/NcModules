// LuaTest.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
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
#include <fstream>
#include "Parser.h"


/**
 * @brief 从文件多钱文件内容
 * @param fpath 文件路径
 * @return 文件内容
*/
std::string GetLuaContent(std::string fpath) {
    // 打开文件
    std::ifstream inputFile(fpath);

    if (!inputFile.is_open()) {
        std::cerr << "无法打开文件 " << fpath << std::endl;
        return "";
    }

    // 读取文件内容到字符串
    std::string fileContents;
    std::string line;

    while (std::getline(inputFile, line)) {
        fileContents += line + "\n";
    }

    // 关闭文件
    inputFile.close();
    return fileContents;
}

int main()
{
    std::string content = "toolCoord =\"default\" \nworkpieceCoord = \"workpiece\"\nmoveJ(1, 2, 3, 4, 5, 6, 0.2, 0, 3)\nmoveJpose(1, 2, 3, 4, 5, 6, 0.3, 0.4, toolCoord, workpieceCoord)\n        moveL(1, 2, 3, 0, 0, 0, 0.2, 0.3, toolCoord, workpieceCoord)\n        moveTcp(1, 2, 3, 0.1, 0.15,toolCoord)";
    std::vector< NcParse::Cmd>  cmds = NcParse::Parser::Parse(content);
    std::cout << "-----------------------结构化数据--------------------" << std::endl;
    for (int i = 0; i < cmds.size(); i++) {
        std::cout << cmds[i].name << " ";
        for (int j = 0; j < cmds[i].params.size(); j++) {
            std::cout << cmds[i].params[j] << " ";
        }
        std::cout << " v:" << cmds[i].v << " a:" << cmds[i].a << " tool:" << cmds[i].tool << " workpiece:" << cmds[i].workpiece << std::endl;
    }
    return 0;
}