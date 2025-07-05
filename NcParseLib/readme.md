# NC指令解析模块
操作文档见 https://alidocs.dingtalk.com/i/nodes/vy20BglGWOe6XmNkT3wNk9alJA7depqY

# 07 NC指令解释器

## 1、 项目位置

*   解释器项目位置(visual studio 2017 vs141)
    

![image](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/res/YvenvJX5b9LVnoyZ/img/f8127fb1-607a-4c31-9954-a5405d762345.png)

## 2、项目配置

![image](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/res/YvenvJX5b9LVnoyZ/img/83541d44-42cd-4ba7-89d3-78ecc9ba0c66.png)

![image](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/res/YvenvJX5b9LVnoyZ/img/50324372-c64a-41eb-8e49-80b43144beed.png)

## 3、结构定义

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
    	std::string tool="";
    
    	// 工件坐标系名称
    	std::string workpiece = "";
    };

## 4、 接口列表

    /**
     * @brief 将用户脚本解释为Cmd数组
     * @param scriptContent 用户编写脚本内容
     * @return 指令集数组
    */
    static std::vector<Cmd> Parse(std::string scriptContent)

## 5、示例

### 5.1 用户编写脚本

    -- 用户自定义控制脚本
    toolCoord ="default" -- 工具坐标系
    workpieceCoord = "default" -- 工件坐标系名称
    
    moveJ(1,2,3,4,5,6)
    moveL(1,2,3,0,0,0,toolCoord,workpieceCoord)
    sleep(100) -- 睡眠100ms
    for v=0,5 do -- 循环5次
    	if (getRobState()==1) then
    		moveJ(1,2,3,4,5,0)
    	else
    		moveL(1,2,3,0,0,0,toolCoord,workpieceCoord)
    	end
    end

### 5.2 解释用户脚本

    ...
    #include "Parser.h"
    
    int main()
    {
        std::string content = GetLuaContent("user.lua");  // 获取用户脚本内容
        std::vector< NcParse::Cmd>  cmds = NcParse::Parser::Parse(content); // 解析用户脚本为Cmd列表
        std::cout << "-----------------------结构化数据--------------------" << std::endl;
        for (int i = 0; i < cmds.size(); i++) {
            std::cout << cmds[i].name << " ";
            for (int j = 0; j < cmds[i].params.size(); j++) {
                std::cout << cmds[i].params[j] << " ";
            }
            std::cout << cmds[i].tool << " " << cmds[i].workpiece << std::endl;
        }
        return 0;
    }

### 5.3 输出

![image](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/res/YvenvJX5b9LVnoyZ/img/901c3e64-729e-4cfd-93bd-bc542b74db08.png)