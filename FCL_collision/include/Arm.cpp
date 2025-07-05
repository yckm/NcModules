#include "Arm.h"


namespace Clash {
	Eigen::Matrix4d Arm::rx(double theta)
	{
		Eigen::Matrix4d mat;
		mat << 1, 0, 0, 0, 0, std::cos(theta), -std::sin(theta), 0, 0, std::sin(theta), std::cos(theta), 0, 0, 0, 0, 1;
		return  mat;
	}

	Eigen::Matrix3d Arm::rx3(double theta)
	{
		Eigen::Matrix3d mat;
		mat << 1, 0, 0,
			0, std::cos(theta), -std::sin(theta),
			0, std::sin(theta), std::cos(theta);
		return mat;
	}

	Eigen::Matrix4d Arm::ry(double theta)
	{
		Eigen::Matrix4d mat;
		mat << std::cos(theta), 0, std::sin(theta), 0, 0, 1, 0, 0, -std::sin(theta), 0, std::cos(theta), 0, 0, 0, 0, 1;
		return  mat;
	}

	Eigen::Matrix3d Arm::ry3(double theta)
	{
		Eigen::Matrix3d mat;
		mat << std::cos(theta), 0, std::sin(theta),
			0, 1, 0,
			-std::sin(theta), 0, std::cos(theta);
		return  mat;
	}

	Eigen::Matrix4d Arm::rz(double theta)
	{
		Eigen::Matrix4d mat;
		mat << std::cos(theta), -std::sin(theta), 0, 0, std::sin(theta), std::cos(theta), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
		return  mat;
	}

	Eigen::Matrix3d Arm::rz3(double theta)
	{
		Eigen::Matrix3d mat;
		mat << std::cos(theta), -std::sin(theta), 0,
			std::sin(theta), std::cos(theta), 0,
			0, 0, 1;
		return  mat;
	}

	Eigen::Matrix4d Arm::matMv(double x, double y, double z)
	{
		Eigen::Matrix4d mat;
		mat << 1, 0, 0, x,
			0, 1, 0, y,
			0, 0, 1, z,
			0, 0, 0, 1;
		return  mat;
	}

	Eigen::Matrix4d Arm::eye()
	{
		Eigen::Matrix4d mat;
		mat << 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;
		return  mat;
	}

	Vec3 Arm::getBias(Eigen::Matrix4d& mat)
	{
		return Vec3(mat(0, 3), mat(1, 3), mat(2, 3));
	}

	Eigen::Matrix4d Arm::zero()
	{
		Eigen::Matrix4d mat;
		mat << 0, 0, 0, 0,
			0, 0, 0, 0,
			0, 0, 0, 0,
			0, 0, 0, 1;
		return  mat;
	}

	Arm::Arm(std::vector<double> arms)
	{
		h1 = arms[0];
		h2 = arms[1];
		h3 = arms[2];
		h4 = arms[3];
		h5 = arms[4];
		h6 = arms[5];
		h7 = arms[6];
		h8 = arms[7];
	}

	std::vector<Eigen::Matrix4d> Arm::GetArmPose(std::vector<double> joints)
	{
		double j1, j2, j3, j4, j5, j6;
		j1 = joints[0];
		j2 = joints[1];
		j3 = joints[2];
		j4 = joints[3];
		j5 = joints[4];
		j6 = joints[5];

		//  再平移
		Eigen::Matrix4d m0 = eye(); // 坐标原点
		Eigen::Matrix4d m1 = matMv(0, 0, h1);//;
		Eigen::Matrix4d m2 = m1 * rz(j1) * matMv(0, h2, 0);
		Eigen::Matrix4d m3 = m2 * ry(j2) * matMv(0, 0, h3);
		Eigen::Matrix4d m4 = m3 * matMv(0, -h4, 0);
		Eigen::Matrix4d m5 = m4 * ry(j3) * matMv(0, 0, h5);
		Eigen::Matrix4d m6 = m5 * matMv(0, h6, 0);
		Eigen::Matrix4d m7 = m6 * ry(j4) * matMv(0, 0, h7);
		Eigen::Matrix4d m8 = m7 * rz(j5) * matMv(0, h8, 0);
		std::vector < Eigen::Matrix4d> mats = { m0, m1, m2, m3, m4, m5, m6, m7, m8 };
		return mats;
	}
}