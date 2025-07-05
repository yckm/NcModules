#pragma once
#include <string>
#include <vector>
#include <Eigen/Core>
#include <chrono>

namespace Utils {
	class Utils
	{
	public:
		/// <summary>
		/// 字符串数组皮拼接为字符串
		/// </summary>
		/// <param name="strs">数组列表</param>
		/// <returns>拼接后的字符串</returns>
		static std::string Arr2String(std::vector< std::string>& strs, std::string spliter = ",") {
			std::string strings("");
			for (auto& s : strs) {
				strings += s + spliter;
			}
			strings.erase(strings.length() - 1, 1);
			return strings;
		}

		/// <summary>
		/// double数组变成string数组
		/// </summary>
		/// <param name="arr">double数组</param>
		/// <returns>字符串数组</returns>
		static std::string DoubleArr2String(std::vector<double> arr)
		{
			if (arr.size() == 0)
			{
				return "[]";
			}

			std::string str = "[";
			for (auto d : arr)
			{
				str += std::to_string(d) + ",";
			}

			str.erase(str.length() - 1, 1);
			return str + "]";
		}

		/**
		 * @创建人:dnp
		 * @简述:单精度数组转双精度数组
		 * @参数:arr 单精度数组
		 * @返回值:算进度速度
		 * **/
		static std::vector<double> toArrd(std::vector<float>& arr) {
			std::vector<double> arrd;
			for (const auto& v : arr) {
				arrd.push_back(v);
			}
			return arrd;
		}

		/// <summary>
		/// 获取当前时间戳(毫秒)
		/// </summary>
		/// <returns>时间戳(毫秒)</returns>
		static long GetTsMs() {
			auto now = std::chrono::high_resolution_clock::now();
			auto nano_time_point = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
			auto epoch = nano_time_point.time_since_epoch();
			return  std::chrono::duration_cast<std::chrono::milliseconds>(epoch).count();
		}

		/**
		 * @创建人 dnp
		 * @简介 分割字符串
		 * @参数 str 待分割的字符串
		 * @参数 pattern 分割符
		 * @返回值 分割后的字符串列表
		 */
		static std::vector<std::string> split(std::string& str, const std::string& pattern) {
			std::vector<std::string> result;
			std::string::size_type begin, end;

			end = str.find(pattern);
			begin = 0;

			while (end != std::string::npos) {
				if (end - begin != 0) {
					result.push_back(str.substr(begin, end - begin));
				}
				begin = end + pattern.size();
				end = str.find(pattern, begin);
			}

			if (begin != str.length()) {
				result.push_back(str.substr(begin));
			}
			return result;
		}

		/**
		* @创建人 dnp
		* @简介 打印4x4的矩阵
		* @参数 tip 文字提示
		* @参数 mat 矩阵
		*/
		static void print4x4(std::string tip, Eigen::Matrix4d& mat) {
			std::cout << "-------------------------------------------------" << std::endl;
			std::cout << tip << ":" << std::endl;
			for (int i = 0; i < 4; i++) {
				std::cout << "[ ";
				for (int j = 0; j < 4; j++) {
					if (j == 3) {
						std::cout << mat(i, j) << " ], ";
					}
					else {
						std::cout << mat(i, j) << " , ";
					}

				}
				std::cout << std::endl;
			}
		}
	};
}