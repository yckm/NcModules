#include "KinematicsWrapper.h"
#include "global.h"
#include <iostream>
#include <map>
#include <Eigen/Dense>
#include "Eigen/Geometry"

namespace wysiwyg {
	namespace Kinematrics
	{
		/// DH参数,单位是米和弧度
		const double m_d1 = 0.165;
		const double m_d4 = 0.164;
		const double m_d5 = 0.126;
		const double m_d6 = 0.113;
		const double m_a2 = -0.608;
		const double m_a3 = -0.566;


		inline    Eigen::Matrix4d RXMat4d(double theta)
		{
			Eigen::Matrix4d RXMat;
			RXMat << 1, 0, 0, 0,
				0, cos(theta), -sin(theta), 0,
				0, sin(theta), cos(theta), 0,
				0, 0, 0, 1;

			return RXMat;
		}


		inline   Eigen::Matrix4d DXMat4d(double dx)
		{
			Eigen::Matrix4d DXMat;
			DXMat << 1, 0, 0, dx,
				0, 1, 0, 0,
				0, 0, 1, 0,
				0, 0, 0, 1;

			return DXMat;
		}

		inline   Eigen::Matrix4d RYMat4d(double theta)
		{
			Eigen::Matrix4d RYMat;
			RYMat << cos(theta), 0, sin(theta), 0,
				0, 1, 0, 0,
				-sin(theta), 0, cos(theta), 0,
				0, 0, 0, 1;

			return RYMat;
		}


		inline  Eigen::Matrix4d DYMat4d(double dy)
		{
			Eigen::Matrix4d DYMat;
			DYMat << 1, 0, 0, 0,
				0, 1, 0, dy,
				0, 0, 1, 0,
				0, 0, 0, 1;

			return DYMat;
		}

		inline  Eigen::Matrix4d RZMat4d(double theta)
		{
			Eigen::Matrix4d RZMat;
			RZMat << cos(theta), -sin(theta), 0, 0,
				sin(theta), cos(theta), 0, 0,
				0, 0, 1, 0,
				0, 0, 0, 1;

			return RZMat;
		}


		inline  Eigen::Matrix4d DZMat4d(double dz)
		{
			Eigen::Matrix4d DZMat;
			DZMat << 1, 0, 0, 0,
				0, 1, 0, 0,
				0, 0, 1, dz,
				0, 0, 0, 1;

			return DZMat;
		}

		// 采用开源的成熟稳定的算法
		// https://github.com/ros-industrial/universal_robot/blob/kinetic-devel/ur_kinematics/src/ur_kin.cpp
		namespace ur_kinematics
		{

			int SIGN(double x)
			{
				return (x > 0) - (x < 0);
			}

			void to_mat44(double* mat4_4, const double* pose, const double* rmat3_3)
			{
				for (int i = 0; i < 3; ++i)
				{
					mat4_4[i * 4 + 0] = rmat3_3[i * 3 + 0];
					mat4_4[i * 4 + 1] = rmat3_3[i * 3 + 1];
					mat4_4[i * 4 + 2] = rmat3_3[i * 3 + 2];
					mat4_4[i * 4 + 3] = pose[i];
				}
				mat4_4[3 * 4 + 0] = 0;
				mat4_4[3 * 4 + 1] = 0;
				mat4_4[3 * 4 + 2] = 0;
				mat4_4[3 * 4 + 3] = 1;
			}

			void from_mat44(const double* mat4_4, double* pose, double* rmat3_3)
			{
				for (int i = 0; i < 3; ++i)
				{
					rmat3_3[i * 3 + 0] = mat4_4[i * 4 + 0];
					rmat3_3[i * 3 + 1] = mat4_4[i * 4 + 1];
					rmat3_3[i * 3 + 2] = mat4_4[i * 4 + 2];
					pose[i] = mat4_4[i * 4 + 3];
				}
			}

			// @param q       The 6 joint values 
			// @param T       The 4x4 end effector pose in row-major ordering
			void forward(const double* q, double* T)
			{
				double s1 = sin(*q), c1 = cos(*q); q++;
				double q234 = *q, s2 = sin(*q), c2 = cos(*q); q++;
				double s3 = sin(*q), c3 = cos(*q); q234 += *q; q++;
				q234 += *q; q++;
				double s5 = sin(*q), c5 = cos(*q); q++;
				double s6 = sin(*q), c6 = cos(*q);
				double s234 = sin(q234), c234 = cos(q234);
				*T = ((c1 * c234 - s1 * s234) * s5) / 2.0 - c5 * s1 + ((c1 * c234 + s1 * s234) * s5) / 2.0; T++;
				*T = (c6 * (s1 * s5 + ((c1 * c234 - s1 * s234) * c5) / 2.0 + ((c1 * c234 + s1 * s234) * c5) / 2.0) -
					(s6 * ((s1 * c234 + c1 * s234) - (s1 * c234 - c1 * s234))) / 2.0); T++;
				*T = (-(c6 * ((s1 * c234 + c1 * s234) - (s1 * c234 - c1 * s234))) / 2.0 -
					s6 * (s1 * s5 + ((c1 * c234 - s1 * s234) * c5) / 2.0 + ((c1 * c234 + s1 * s234) * c5) / 2.0)); T++;
				*T = ((m_d5 * (s1 * c234 - c1 * s234)) / 2.0 - (m_d5 * (s1 * c234 + c1 * s234)) / 2.0 -
					m_d4 * s1 + (m_d6 * (c1 * c234 - s1 * s234) * s5) / 2.0 + (m_d6 * (c1 * c234 + s1 * s234) * s5) / 2.0 -
					m_a2 * c1 * c2 - m_d6 * c5 * s1 - m_a3 * c1 * c2 * c3 + m_a3 * c1 * s2 * s3); T++;
				*T = c1 * c5 + ((s1 * c234 + c1 * s234) * s5) / 2.0 + ((s1 * c234 - c1 * s234) * s5) / 2.0; T++;
				*T = (c6 * (((s1 * c234 + c1 * s234) * c5) / 2.0 - c1 * s5 + ((s1 * c234 - c1 * s234) * c5) / 2.0) +
					s6 * ((c1 * c234 - s1 * s234) / 2.0 - (c1 * c234 + s1 * s234) / 2.0)); T++;
				*T = (c6 * ((c1 * c234 - s1 * s234) / 2.0 - (c1 * c234 + s1 * s234) / 2.0) -
					s6 * (((s1 * c234 + c1 * s234) * c5) / 2.0 - c1 * s5 + ((s1 * c234 - c1 * s234) * c5) / 2.0)); T++;
				*T = ((m_d5 * (c1 * c234 - s1 * s234)) / 2.0 - (m_d5 * (c1 * c234 + s1 * s234)) / 2.0 + m_d4 * c1 +
					(m_d6 * (s1 * c234 + c1 * s234) * s5) / 2.0 + (m_d6 * (s1 * c234 - c1 * s234) * s5) / 2.0 + m_d6 * c1 * c5 -
					m_a2 * c2 * s1 - m_a3 * c2 * c3 * s1 + m_a3 * s1 * s2 * s3); T++;
				*T = ((c234 * c5 - s234 * s5) / 2.0 - (c234 * c5 + s234 * s5) / 2.0); T++;
				*T = ((s234 * c6 - c234 * s6) / 2.0 - (s234 * c6 + c234 * s6) / 2.0 - s234 * c5 * c6); T++;
				*T = (s234 * c5 * s6 - (c234 * c6 + s234 * s6) / 2.0 - (c234 * c6 - s234 * s6) / 2.0); T++;
				*T = (m_d1 + (m_d6 * (c234 * c5 - s234 * s5)) / 2.0 + m_a3 * (s2 * c3 + c2 * s3) + m_a2 * s2 -
					(m_d6 * (c234 * c5 + s234 * s5)) / 2.0 - m_d5 * c234); T++;
				*T = 0.0; T++; *T = 0.0; T++; *T = 0.0; T++; *T = 1.0;
			}

			// @param q       The 6 joint values 
			// @param Ti      The 4x4 link i pose in row-major ordering. If NULL, nothing is stored.
			void forward_all(const double* q, double* T1, double* T2, double* T3, double* T4, double* T5, double* T6)
			{
				double s1 = sin(*q), c1 = cos(*q); q++; // q1
				double q23 = *q, q234 = *q, s2 = sin(*q), c2 = cos(*q); q++; // q2
				double s3 = sin(*q), c3 = cos(*q); q23 += *q; q234 += *q; q++; // q3
				q234 += *q; q++; // q4
				double s5 = sin(*q), c5 = cos(*q); q++; // q5
				double s6 = sin(*q), c6 = cos(*q); // q6
				double s23 = sin(q23), c23 = cos(q23);
				double s234 = sin(q234), c234 = cos(q234);

				if (T1 != NULL)
				{
					*T1 = c1; T1++;
					*T1 = 0; T1++;
					*T1 = s1; T1++;
					*T1 = 0; T1++;
					*T1 = s1; T1++;
					*T1 = 0; T1++;
					*T1 = -c1; T1++;
					*T1 = 0; T1++;
					*T1 = 0; T1++;
					*T1 = 1; T1++;
					*T1 = 0; T1++;
					*T1 = m_d1; T1++;
					*T1 = 0; T1++;
					*T1 = 0; T1++;
					*T1 = 0; T1++;
					*T1 = 1; T1++;
				}

				if (T2 != NULL)
				{
					*T2 = c1 * c2; T2++;
					*T2 = -c1 * s2; T2++;
					*T2 = s1; T2++;
					*T2 = m_a2 * c1 * c2; T2++;
					*T2 = c2 * s1; T2++;
					*T2 = -s1 * s2; T2++;
					*T2 = -c1; T2++;
					*T2 = m_a2 * c2 * s1; T2++;
					*T2 = s2; T2++;
					*T2 = c2; T2++;
					*T2 = 0; T2++;
					*T2 = m_d1 + m_a2 * s2; T2++;
					*T2 = 0; T2++;
					*T2 = 0; T2++;
					*T2 = 0; T2++;
					*T2 = 1; T2++;
				}

				if (T3 != NULL)
				{
					*T3 = c23 * c1; T3++;
					*T3 = -s23 * c1; T3++;
					*T3 = s1; T3++;
					*T3 = c1 * (m_a3 * c23 + m_a2 * c2); T3++;
					*T3 = c23 * s1; T3++;
					*T3 = -s23 * s1; T3++;
					*T3 = -c1; T3++;
					*T3 = s1 * (m_a3 * c23 + m_a2 * c2); T3++;
					*T3 = s23; T3++;
					*T3 = c23; T3++;
					*T3 = 0; T3++;
					*T3 = m_d1 + m_a3 * s23 + m_a2 * s2; T3++;
					*T3 = 0; T3++;
					*T3 = 0; T3++;
					*T3 = 0; T3++;
					*T3 = 1; T3++;
				}

				if (T4 != NULL)
				{
					*T4 = c234 * c1; T4++;
					*T4 = s1; T4++;
					*T4 = s234 * c1; T4++;
					*T4 = c1 * (m_a3 * c23 + m_a2 * c2) + m_d4 * s1; T4++;
					*T4 = c234 * s1; T4++;
					*T4 = -c1; T4++;
					*T4 = s234 * s1; T4++;
					*T4 = s1 * (m_a3 * c23 + m_a2 * c2) - m_d4 * c1; T4++;
					*T4 = s234; T4++;
					*T4 = 0; T4++;
					*T4 = -c234; T4++;
					*T4 = m_d1 + m_a3 * s23 + m_a2 * s2; T4++;
					*T4 = 0; T4++;
					*T4 = 0; T4++;
					*T4 = 0; T4++;
					*T4 = 1; T4++;
				}

				if (T5 != NULL)
				{
					*T5 = s1 * s5 + c234 * c1 * c5; T5++;
					*T5 = -s234 * c1; T5++;
					*T5 = c5 * s1 - c234 * c1 * s5; T5++;
					*T5 = c1 * (m_a3 * c23 + m_a2 * c2) + m_d4 * s1 + m_d5 * s234 * c1; T5++;
					*T5 = c234 * c5 * s1 - c1 * s5; T5++;
					*T5 = -s234 * s1; T5++;
					*T5 = -c1 * c5 - c234 * s1 * s5; T5++;
					*T5 = s1 * (m_a3 * c23 + m_a2 * c2) - m_d4 * c1 + m_d5 * s234 * s1; T5++;
					*T5 = s234 * c5; T5++;
					*T5 = c234; T5++;
					*T5 = -s234 * s5; T5++;
					*T5 = m_d1 + m_a3 * s23 + m_a2 * s2 - m_d5 * c234; T5++;
					*T5 = 0; T5++;
					*T5 = 0; T5++;
					*T5 = 0; T5++;
					*T5 = 1; T5++;
				}

				if (T6 != NULL)
				{
					*T6 = c6 * (s1 * s5 + c234 * c1 * c5) - s234 * c1 * s6; T6++;
					*T6 = -s6 * (s1 * s5 + c234 * c1 * c5) - s234 * c1 * c6; T6++;
					*T6 = c5 * s1 - c234 * c1 * s5; T6++;
					*T6 = m_d6 * (c5 * s1 - c234 * c1 * s5) + c1 * (m_a3 * c23 + m_a2 * c2) + m_d4 * s1 + m_d5 * s234 * c1; T6++;
					*T6 = -c6 * (c1 * s5 - c234 * c5 * s1) - s234 * s1 * s6; T6++;
					*T6 = s6 * (c1 * s5 - c234 * c5 * s1) - s234 * c6 * s1; T6++;
					*T6 = -c1 * c5 - c234 * s1 * s5; T6++;
					*T6 = s1 * (m_a3 * c23 + m_a2 * c2) - m_d4 * c1 - m_d6 * (c1 * c5 + c234 * s1 * s5) + m_d5 * s234 * s1; T6++;
					*T6 = c234 * s6 + s234 * c5 * c6; T6++;
					*T6 = c234 * c6 - s234 * c5 * s6; T6++;
					*T6 = -s234 * s5; T6++;
					*T6 = m_d1 + m_a3 * s23 + m_a2 * s2 - m_d5 * c234 - m_d6 * s234 * s5; T6++;
					*T6 = 0; T6++;
					*T6 = 0; T6++;
					*T6 = 0; T6++;
					*T6 = 1; T6++;
				}
			}

			// @param T       The 4x4 end effector pose in row-major ordering
			// @param q_sols  An 8x6 array of doubles returned, all angles should be in [0,2*PI)
			// @param q6_des  An optional parameter which designates what the q6 value should take
			//                in case of an infinite solution on that joint.
			// @return        Number of solutions found (maximum of 8)
			int inverse(const double* T, double* q_sols, double q6_des)
			{
				int num_sols = 0;
				double T02 = -*T; T++; double T00 = *T; T++; double T01 = *T; T++; double T03 = -*T; T++;
				double T12 = -*T; T++; double T10 = *T; T++; double T11 = *T; T++; double T13 = -*T; T++;
				double T22 = *T; T++; double T20 = -*T; T++; double T21 = -*T; T++; double T23 = *T;

				////////////////////////////// shoulder rotate joint (q1) //////////////////////////////
				double q1[2];
				{
					double A = m_d6 * T12 - T13;
					double B = m_d6 * T02 - T03;
					double R = A * A + B * B;
					if (fabs(A) < ZERO_THRESH)
					{
						double div;
						if (fabs(fabs(m_d4) - fabs(B)) < ZERO_THRESH)
						{
							div = -SIGN(m_d4) * SIGN(B);
						}
						else
						{
							div = -m_d4 / B;
						}
						double arcsin = asin(div);
						if (fabs(arcsin) < ZERO_THRESH)
						{
							arcsin = 0.0;
						}
						if (arcsin < 0.0)
						{
							q1[0] = arcsin + 2.0 * PI;
						}
						else
						{
							q1[0] = arcsin;
						}
						q1[1] = PI - arcsin;
					}
					else if (fabs(B) < ZERO_THRESH)
					{
						double div;
						if (fabs(fabs(m_d4) - fabs(A)) < ZERO_THRESH)
						{
							div = SIGN(m_d4) * SIGN(A);
						}
						else
						{
							div = m_d4 / A;
						}
						double arccos = acos(div);
						q1[0] = arccos;
						q1[1] = 2.0 * PI - arccos;
					}
					else if (m_d4 * m_d4 > R)
					{
						return num_sols;
					}
					else
					{
						double arccos = acos(m_d4 / sqrt(R));
						double arctan = atan2(-B, A);
						double pos = arccos + arctan;
						double neg = -arccos + arctan;
						if (fabs(pos) < ZERO_THRESH)
						{
							pos = 0.0;
						}
						if (fabs(neg) < ZERO_THRESH)
						{
							neg = 0.0;
						}
						if (pos >= 0.0)
						{
							q1[0] = pos;
						}
						else
						{
							q1[0] = 2.0 * PI + pos;
						}
						if (neg >= 0.0)
						{
							q1[1] = neg;
						}
						else
						{
							q1[1] = 2.0 * PI + neg;
						}
					}
				}
				////////////////////////////////////////////////////////////////////////////////

				////////////////////////////// wrist 2 joint (q5) //////////////////////////////
				double q5[2][2];
				{
					for (int i = 0; i < 2; i++)
					{
						double numer = (T03 * sin(q1[i]) - T13 * cos(q1[i]) - m_d4);
						double div;
						if (fabs(fabs(numer) - fabs(m_d6)) < ZERO_THRESH)
						{
							div = SIGN(numer) * SIGN(m_d6);
						}
						else
						{
							div = numer / m_d6;
						}
						double arccos = acos(div);
						q5[i][0] = arccos;
						q5[i][1] = 2.0 * PI - arccos;
					}
				}
				////////////////////////////////////////////////////////////////////////////////

				{
					for (int i = 0; i < 2; i++)
					{
						for (int j = 0; j < 2; j++)
						{
							double c1 = cos(q1[i]), s1 = sin(q1[i]);
							double c5 = cos(q5[i][j]), s5 = sin(q5[i][j]);
							double q6;
							////////////////////////////// wrist 3 joint (q6) //////////////////////////////
							if (fabs(s5) < ZERO_THRESH)
							{
								q6 = q6_des;
							}
							else
							{
								q6 = atan2(SIGN(s5) * -(T01 * s1 - T11 * c1), SIGN(s5) * (T00 * s1 - T10 * c1));
								if (fabs(q6) < ZERO_THRESH)
								{
									q6 = 0.0;
								}
								if (q6 < 0.0)
								{
									q6 += 2.0 * PI;
								}
							}
							////////////////////////////////////////////////////////////////////////////////

							double q2[2], q3[2], q4[2];
							///////////////////////////// RRR joints (q2,q3,q4) ////////////////////////////
							double c6 = cos(q6), s6 = sin(q6);
							double x04x = -s5 * (T02 * c1 + T12 * s1) - c5 * (s6 * (T01 * c1 + T11 * s1) - c6 * (T00 * c1 + T10 * s1));
							double x04y = c5 * (T20 * c6 - T21 * s6) - T22 * s5;
							double p13x = m_d5 * (s6 * (T00 * c1 + T10 * s1) + c6 * (T01 * c1 + T11 * s1)) - m_d6 * (T02 * c1 + T12 * s1) + T03 * c1 + T13 * s1;
							double p13y = T23 - m_d1 - m_d6 * T22 + m_d5 * (T21 * c6 + T20 * s6);

							double c3 = (p13x * p13x + p13y * p13y - m_a2 * m_a2 - m_a3 * m_a3) / (2.0 * m_a2 * m_a3);
							if (fabs(fabs(c3) - 1.0) < ZERO_THRESH)
							{
								c3 = SIGN(c3);
							}
							else if (fabs(c3) > 1.0)
							{
								// TODO NO SOLUTION
								continue;
							}
							double arccos = acos(c3);
							q3[0] = arccos;
							q3[1] = 2.0 * PI - arccos;
							double denom = m_a2 * m_a2 + m_a3 * m_a3 + 2 * m_a2 * m_a3 * c3;
							double s3 = sin(arccos);
							double A = (m_a2 + m_a3 * c3), B = m_a3 * s3;
							q2[0] = atan2((A * p13y - B * p13x) / denom, (A * p13x + B * p13y) / denom);
							q2[1] = atan2((A * p13y + B * p13x) / denom, (A * p13x - B * p13y) / denom);
							double c23_0 = cos(q2[0] + q3[0]);
							double s23_0 = sin(q2[0] + q3[0]);
							double c23_1 = cos(q2[1] + q3[1]);
							double s23_1 = sin(q2[1] + q3[1]);
							q4[0] = atan2(c23_0 * x04y - s23_0 * x04x, x04x * c23_0 + x04y * s23_0);
							q4[1] = atan2(c23_1 * x04y - s23_1 * x04x, x04x * c23_1 + x04y * s23_1);
							////////////////////////////////////////////////////////////////////////////////
							for (int k = 0; k < 2; k++)
							{
								if (fabs(q2[k]) < ZERO_THRESH)
								{
									q2[k] = 0.0;
								}
								else if (q2[k] < 0.0)
								{
									q2[k] += 2.0 * PI;
								}
								if (fabs(q4[k]) < ZERO_THRESH)
								{
									q4[k] = 0.0;
								}
								else if (q4[k] < 0.0)
								{
									q4[k] += 2.0 * PI;
								}
								q_sols[num_sols * 6 + 0] = q1[i];    q_sols[num_sols * 6 + 1] = q2[k];
								q_sols[num_sols * 6 + 2] = q3[k];    q_sols[num_sols * 6 + 3] = q4[k];
								q_sols[num_sols * 6 + 4] = q5[i][j]; q_sols[num_sols * 6 + 5] = q6;
								num_sols++;
							}

						}
					}
				}
				return num_sols;
			}

			void fixAngles(double& dF)
			{
				if (dF > 360)
				{
					dF -= 360;
				}
				else if (dF < -360)
				{
					dF += 360;
				}
				else if (fabs(dF - 360) < ZERO_THRESH)
				{
					dF = 0;
				}
			}

			Eigen::Vector3d rotationMatrixToEulerAngles(Eigen::Matrix3d& R, bool bNeedChg)
			{
				Eigen::Matrix3d R2 = R;
				if (bNeedChg)
				{
					R2 << -R(0, 1), -R(0, 2), R(0, 0),
						-R(1, 1), -R(1, 2), R(1, 0),
						-R(2, 1), -R(2, 2), R(2, 0);
				}
				double sy = sqrt(R2(0, 0) * R2(0, 0) + R2(1, 0) * R2(1, 0));
				bool singular = sy < 1e-6;
				double x, y, z;
				// 不等0
				if (!singular)
				{
					x = atan2(R2(2, 1), R2(2, 2));
					y = atan2(-R2(2, 0), sy);
					z = atan2(R2(1, 0), R2(0, 0));
				}
				else
				{
					x = atan2(-R2(1, 2), R2(1, 1));
					y = atan2(-R2(2, 0), sy);
					z = 0;
				}
				return { x, y, z };
			}

			Eigen::Matrix3d eulerAnglesToRotationMatrix(Eigen::Vector3d& theta, bool bNeedChg)
			{
				Eigen::Matrix3d R_x;    // 计算旋转矩阵的X分量
				R_x <<
					1, 0, 0,
					0, cos(theta[0]), -sin(theta[0]),
					0, sin(theta[0]), cos(theta[0]);

				Eigen::Matrix3d R_y;    // 计算旋转矩阵的Y分量
				R_y <<
					cos(theta[1]), 0, sin(theta[1]),
					0, 1, 0,
					-sin(theta[1]), 0, cos(theta[1]);

				Eigen::Matrix3d R_z;    // 计算旋转矩阵的Z分量
				R_z <<
					cos(theta[2]), -sin(theta[2]), 0,
					sin(theta[2]), cos(theta[2]), 0,
					0, 0, 1;

				Eigen::Matrix3d R = R_z * R_y * R_x;

				// 算法算出来的旋转矩阵和常规的有区别，要转换一下才能正常的算出正确的欧拉角
				if (bNeedChg)
				{
					Eigen::Matrix3d R2;
					R2 << R(0, 2), -R(0, 0), -R(0, 1),
						R(1, 2), -R(1, 0), -R(1, 1),
						R(2, 2), -R(2, 0), -R(2, 1);
					return R2;
				}
				return R;
			}


			std::vector<double> BasePoseToOther(std::vector<double>& vBasePose, std::vector<double>& vTPose)
			{
				Eigen::Matrix4d MT = DXMat4d(vTPose[0]) * DYMat4d(vTPose[1]) * DZMat4d(vTPose[2])
					* RZMat4d(vTPose[5]) * RYMat4d(vTPose[4]) * RXMat4d(vTPose[3]);

				Eigen::Matrix4d mBase = DXMat4d(vBasePose[0]) * DYMat4d(vBasePose[1]) * DZMat4d(vBasePose[2])
					* RZMat4d(vBasePose[5]) * RYMat4d(vBasePose[4]) * RXMat4d(vBasePose[3]);
				Eigen::Matrix4d T = MT.inverse() * mBase;

				Eigen::Vector4d vBase(vBasePose[0], vBasePose[1], vBasePose[2], 1.0);
				Eigen::Vector4d result = MT.inverse() * vBase;

				Eigen::Matrix3d RMatrix;
				RMatrix << T(0, 0), T(0, 1), T(0, 2),
					T(1, 0), T(1, 1), T(1, 2),
					T(2, 0), T(2, 1), T(2, 2);
				Eigen::Vector3d eulerAngle = rotationMatrixToEulerAngles(RMatrix, false);

				std::vector<double> poseVec;
				poseVec.push_back(result(0)); poseVec.push_back(result(1)); poseVec.push_back(result(2));
				poseVec.push_back(eulerAngle(0)); poseVec.push_back(eulerAngle(1)); poseVec.push_back(eulerAngle(2));
				return poseVec;
			}


		}

		bool KinematicsWrapper::FKinematics(std::vector<double>& injointsDeg, std::vector<double>& outPose)
		{
			if (injointsDeg.size() == 6)
			{
				/// 新松的机械臂和一般的UR5不同，角度上需要特殊处理一下
				double q[6] = {
					injointsDeg[0] * DEG2RAD,
					(injointsDeg[1] - 90) * DEG2RAD,
					injointsDeg[2] * DEG2RAD,
					(injointsDeg[3] - 90) * DEG2RAD,
					injointsDeg[4] * DEG2RAD,
					injointsDeg[5] * DEG2RAD
				};

				// 正解算
				double T[16] = { 0 };
				ur_kinematics::forward(q, T);

				// 求出欧拉角和位置信息
				double pose[3] = { 0 };
				double rmat3_3[9] = { 0 };
				ur_kinematics::from_mat44(T, pose, rmat3_3);

				// 把旋转矩阵转为欧拉角
				Eigen::Matrix3d R;
				R << rmat3_3[0], rmat3_3[1], rmat3_3[2],
					rmat3_3[3], rmat3_3[4], rmat3_3[5],
					rmat3_3[6], rmat3_3[7], rmat3_3[8];
				Eigen::Vector3d euler = ur_kinematics::rotationMatrixToEulerAngles(R, true);

				// 米转为毫米
				outPose.push_back(pose[0] * 1000);
				outPose.push_back(pose[1] * 1000);
				outPose.push_back(pose[2] * 1000);

				// 弧度转角度
				outPose.push_back(euler[0] * RAD2DEG);
				outPose.push_back(euler[1] * RAD2DEG);
				outPose.push_back(euler[2] * RAD2DEG);

				return true;
			}
			return false;
		}

		bool KinematicsWrapper::IKinematics(std::vector<double>& inPose, std::vector<std::vector<double>>& outjointsDeg)
		{
			if (inPose.size() == 6)
			{
				// 合成齐次矩阵
				double pose[3] = { inPose[0] * 0.001,inPose[1] * 0.001,inPose[2] * 0.001 };
				Eigen::Vector3d theta = { inPose[3] * DEG2RAD,inPose[4] * DEG2RAD,inPose[5] * DEG2RAD };
				double eerot[9] = { 0 };
				Eigen::Matrix3d R = ur_kinematics::eulerAnglesToRotationMatrix(theta, true);
				for (int i = 0; i < 3; ++i)
				{
					for (int j = 0; j < 3; ++j)
					{
						eerot[i * 3 + j] = R(i, j);
					}
				}
				double T[16] = { 0 };
				ur_kinematics::to_mat44(T, pose, eerot);

				// 逆解算返回的角度范围是[0,2*PI)
				double q_sols[8 * 6] = { 0 };
				int num_sols = num_sols = ur_kinematics::inverse(T, q_sols, 0);
				for (int i = 0; i < num_sols; i++)
				{
					std::vector<double> vOne;
					for (int j = 0; j < 6; j++)
					{
						double dDeg = q_sols[i * 6 + j] * RAD2DEG;
						// 关节1和关节4要特殊处理一下
						if (j == 1 || j == 3)
						{
							dDeg += 90;
							ur_kinematics::fixAngles(dDeg);
						}
						// 关节3有角度限制[-160到160], 不满足要求的不能作为解
						//else if (j == 2)
						//{
						//	if (dDeg > 160 && dDeg < 200)
						//	{
						//		break;
						//	}
						//	if (dDeg >= 200)
						//	{
						//		dDeg -= 360;
						//	}
						//}
						vOne.push_back(dDeg);
					}
					if (vOne.size() == 6)
					{
						outjointsDeg.push_back(vOne);
					}
				}
				return num_sols;
			}
			return false;
		}

		bool KinematicsWrapper::IKinematics(std::vector<double>& inPose, std::vector<double>& inCurrjointsDeg, std::vector<double>& outjointsDeg)
		{
			if (inPose.size() != 6)
			{
				return false;
			}
			std::vector<std::vector<double>> AlloutjointsDeg;
			IKinematics(inPose, AlloutjointsDeg);


			if (inCurrjointsDeg.size() == 6)
			{
				// 给定每个关节的权重
				std::vector<double> vWeight = {0.6,0.5,0.5,0.2,0.1,0.1 };

				std::map<int, std::vector<double>> mJoints;
				for (auto& p : AlloutjointsDeg)
				{
					int nTotal = 0;
					for (int i = 0; i < 6; ++i)
					{
						// 计算对应角度的差值
						double dTemp = p[i];
						double dd = fabs(dTemp - inCurrjointsDeg[i]);
						if (dd > 180)
						{
							dTemp -= 360;
							ur_kinematics::fixAngles(dTemp);
							dd = fabs(dTemp - inCurrjointsDeg[i]);
						}
						ur_kinematics::fixAngles(dd);
						dd *= vWeight[i];
						dd *= dd;
						nTotal += dd;
					}
					mJoints[nTotal] = p;
				}
				// 选加权和最小的
				if (mJoints.size() > 0)
				{
					outjointsDeg = mJoints.begin()->second;
					for (int i = 0; i < 6; ++i)
					{
						double d1 = fabs(inCurrjointsDeg[i] - outjointsDeg[i]);
						if (d1 > 180)
						{
							outjointsDeg[i] -= 360;
							ur_kinematics::fixAngles(outjointsDeg[i]);
						}
					}
					return true;
				}
			}


			return false;
		}
	}
	}