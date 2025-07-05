#pragma once
#include <Eigen/Dense>
#include <Eigen/StdVector>

namespace Clash {
	using Matrix3d = Eigen::Matrix3<double>;
	using Vector3d = Eigen::Vector3<double>;

	struct Point {
		float x;
		float y;
		float z;
		std::string name = "";

		/**
		 * @创建人:dnp
		 * @简述:计算点到原点的长度
		 * @返回值:点到原点长度
		 * **/
		float length() {
			return sqrt(x * x + y * y + z * z);
		}
	};

	struct Cylinder {
		float r;
		Point p0;
		Point p1;

		/**
		 * @创建人:dnp
		 * @简述:计算方向向量
		 * @返回值: 方向向量
		 * **/
		Point direction() {
			return Point{ p0.x - p1.x,p0.y - p1.y,p0.z - p1.z };
		}

		float length() {
			float dx = p0.x - p1.x;
			float dy = p0.y - p1.y;
			float dz = p0.z - p1.z;
			return sqrt(dx * dx + dy * dy + dz * dz);
		}
	};

	/**
 * @创建人:dnp
 * @创建日期:2023-11-14
 * @简述: 圆柱模型
 * @返回值:
 * **/
	class CylinderModel {
	public:
		CylinderModel(double radisus, double height, std::vector<std::vector<double>> trans) {
			this->height = height;
			this->radius = radisus;
			this->rotation << trans[0][0], trans[0][1], trans[0][2],
				trans[1][0], trans[1][1], trans[1][2],
				trans[2][0], trans[2][1], trans[2][2];
			this->mv[0] = trans[0][3];
			this->mv[1] = trans[1][3];
			this->mv[2] = trans[2][3];
		}
		double radius;
		double height;

		// 旋转矩阵
		Matrix3d rotation;

		// 平移向量
		Vector3d mv;
	};
}
