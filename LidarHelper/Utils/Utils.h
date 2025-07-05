#pragma once
#include <string>
#include <vector>

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
	};
}