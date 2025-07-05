#pragma once
#include <string>
#include <vector>
#include "Utils.h"
#include <iostream>


namespace Utils {
	class LogClient
	{
	private:
		bool inited;
	public:
		/// <summary>
		/// 创建数据库连接
		/// </summary>
		bool InitClient() {
			/*
			if (inited) {
				return true;
			}
			mysql_init(&logclient);
			MYSQL* ret = mysql_real_connect(&logclient, "127.0.0.1", "dev", "dev123", "db_test", 3306, NULL, 0);
			if (ret == NULL) {
				printf("数据库连接失败！失败原因：%s\n", mysql_error(&logclient));
				return false;
			}

			inited = true;
			*/
			return true;
		}

		/// <summary>
		/// 记录日志
		/// </summary>
		/// <param name="p_type">日志类型</param>
		/// <param name="p_msg">日志内容</param>
		/// <param name="p_msg">flag</param>
		void log(std::string p_type, std::string p_msg, std::string flag) {
			/*
			std::string sql = "INSERT INTO `db_test`.`rob_logs`(`type`, `msg`,`flag`,`ts`) VALUES ('" + p_type + "','" + p_msg + "','" + flag + "', " + std::to_string(Utils::GetTsMs()) + ");";
			mysql_query(&logclient, sql.c_str());
			if (mysql_query(&logclient, "COMMIT") != 0) {
				printf("Commit failed: %s\n", mysql_error(&logclient));
				// Handle commit failure
			}
			*/
			std::cout << "Type=>" << p_type << "  Msg=>" << p_msg << "  Flag=>" << flag << std::endl;
		}
	};
}