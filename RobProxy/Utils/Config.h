#pragma once
#include <map>
#include <fstream>
#include <sstream>
#include <string>
#include <json.hpp>
#include <iostream>
#include <glog/logging.h>
#include <Eigen/Core>
#include "../Utils/Utils.h"

namespace Utils {	

	/**
	 * @创建人 dnp
	 * @简介 相机模型
	 */
	struct Camera {
		// 相机坐在的机械臂
		int robId;

		// tcp到相机的转换矩阵
		Eigen::Matrix4d mat;
	};

	class ConfigDict
	{
	
		public:
#pragma region 相机变换矩阵
			/**
			 * @创建人 dnp
			 * @简介 换成变换矩阵
			 * @参数 key 键 
			 * @参数 robId 机器人id
			 * @参数 arr 4x4变换矩阵
			 */
			void update(std::string key,int robId, std::vector<std::vector<double>>& arr) {
				Eigen::Matrix4d m4;
				for (int i = 0; i < 4; i++) {
					for (int j = 0; j < 4; j++) {
						m4(i, j) = arr[i][j];
					}
				}

				Camera c = { robId,m4 };
				x4s[key] = c;
			}

			/**
			 * @创建人 dnp
			 * @简介 获取变换矩阵
			 * @参数 key 键
			 * @返回值 变换矩阵
			 */
			Camera get(std::string key) {
				return x4s[key];
			}
#pragma endregion

#pragma region 范围
			/**
			 * @创建人 dnp
			 * @简介 更新一维数组字典
			 * @参数 key 键
			 * @参数 arr 一维数组
			 */
			void update(std::string key, std::vector<double> arr) {
				d1array[key] = arr;
			}

			/**
			 * @创建人 dnp
			 * @简介 获取一维数组
			 * @参数 key 键
			 * @返回值 一维数组
			 */
			std::vector<double> get1Arr(std::string key) {
				return d1array[key];
			}
#pragma endregion
		private:
			std::map < std::string, Camera> x4s; // 相机变换矩阵
			std::map<std::string, std::vector<double>> d1array; // 一维数组
	};

	class  Config
	{
	private:
		static ConfigDict* dic ;
	
	public:
		/**
		 * @创建人 dnp
		 * @简介 初始化配置文件
		 */
		void static  init();

		/**
		 * @创建人 dnp
		 * @简介 获取相机
		 * @参数 key 键
		 * @返回值 
		 */
		static Camera getCamera(std::string key);

		/**
		 * @创建人 dnp
		 * @简介 获取一维数组
		 * @参数 key 键
		 * @返回值 一维数组
		 */
		static std::vector<double> get1Arr(std::string key);
	};

	
}
