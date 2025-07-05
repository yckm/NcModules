/*****************************************************************
@文件名称:global.h
@创建人:张平
@创建日期:2023-11-2
@简述:用来存放全局变量和全局结构体的定义
@更改历史:
日期:         作者:             简述:
*******************************************************************/
#pragma once
#include <vector>
namespace wysiwyg {
	namespace Kinematrics
	{
		extern const double PI = 3.141592653589793238462643383279502884197169399375105820974944592307816406L;
		extern const double ZERO_THRESH = 0.00000001;
		/// the value pi/180
		extern const double DEG2RAD = PI / 180;
		/// the value 180/pi
		extern const double RAD2DEG = 180 / PI;


		/**
		@简单描述:用来表示机器人位姿信息的结构体
		**/
		struct TAG_POSE6D
		{
			TAG_POSE6D()
			{

			}
			TAG_POSE6D(double x, double y, double z, double rx, double ry, double rz)
			{
				X = x;
				Y = y;
				Z = z;
				RX = rx;
				RY = ry;
				RZ = rz;
			}

			std::vector<double> ToVector()
			{
				std::vector<double> ret;
				ret.push_back(X);
				ret.push_back(Y);
				ret.push_back(Z);
				ret.push_back(RX);
				ret.push_back(RY);
				ret.push_back(RZ);
				return ret;
			}

			bool IsVaildIK()
			{
				if (fabs(X) > ZERO_THRESH)
				{
					return true;
				}
				if (fabs(Y) > ZERO_THRESH)
				{
					return true;
				}
				if (fabs(Z) > ZERO_THRESH)
				{
					return true;
				}
				if (fabs(RX) > ZERO_THRESH)
				{
					return true;
				}
				if (fabs(RY) > ZERO_THRESH)
				{
					return true;
				}
				if (fabs(RZ) > ZERO_THRESH)
				{
					return true;
				}
				return false;
			}
			/// 单位是米
			double X = 0;
			double Y = 0;
			double Z = 0;

			// 单位是弧度
			double RX = 0;
			double RY = 0;
			double RZ = 0;
		};

		struct TAG_JOINT_INFO
		{
			double dTime = 0;
			double dRad = 0;
			double dVelocity = 0;
			double dAccel = 0;
		};

		struct TAG_SEX_JOINT_INFO
		{
			TAG_JOINT_INFO sexInfo[6];
		};


		enum EN_FUNC_TYPE
		{
			MOVEJ = 0,
			MOVEL,
			SPEEDJ,
			SPEEDL,
		};

		struct TAG_TASK_INFO
		{
			TAG_TASK_INFO()
			{
				nID = 0;
				nType = MOVEJ;
			}
			int32_t nID;
			EN_FUNC_TYPE nType;
			std::vector<double> vB;
			std::vector<double> vE;
			double dMaxV;
			double dMaxA;
		};
	}
}