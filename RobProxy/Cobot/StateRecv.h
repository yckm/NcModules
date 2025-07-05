#pragma once
#define WIN32_LEAN_AND_MEAN

#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <thread>
#include <mutex>
#include "../Models.h"
#include "../Utils/Utils.h"
#include "../Proxy.h"
#include "../Models.h"

using namespace std;

// Need to link with Ws2_32.lib, Mswsock.lib, and Advapi32.lib.
#pragma comment (lib, "Ws2_32.lib")
#pragma comment (lib, "Mswsock.lib")
//#pragma comment (lib, "AdvApi32.lib")

#define COBOTAPI __declspec(dllexport)

namespace Cobot {
	/// <summary>
	/// 接收来自机器人推送的机器人状态信息
	/// </summary>
	class COBOTAPI StateRecv
	{
	private:
		/// 机械臂ID
		int robId;
		/// <summary>
		/// Nc port
		/// </summary>
		std::string tcpServerPort;

		/// <summary>
		/// nc ip
		/// </summary>
		std::string tcpServerIp;

		/// <summary>
		/// 最近更新的数组 (使用两组数组交替使用替换锁)
		/// </summary>
		int newArrIdx = 0;

		/// <summary>
		/// 关节角度
		/// </summary>
		float joints1[6] = { 0 };

		/// <summary>
		/// tcp位姿
		/// </summary>
		float tcps1[6] = { 0 };

		/// <summary>
		/// 关节角度
		/// </summary>
		float joints2[6] = { 0 };

		/// <summary>
		/// tcp位姿
		/// </summary>
		float tcps2[6] = { 0 };

		/// <summary>
		/// 最近一次NC推送数据时间戳(毫秒)
		/// </summary>
		long lastPushTsMs = 0;

		SOCKET ConnectSocket = INVALID_SOCKET;

	public:
		StateRecv(int p_robId, std::string p_ip, std::string p_port) {
			tcpServerPort = p_port;
			tcpServerIp = p_ip;
			robId = p_robId;
		}

		/**
		 * @创建人:dnp
		 * @创建日期:2023-10-19
		 * @简述:从Nc接收机械臂的各状态
		 * @返回值:
		 * **/
		void ReceiveFromServer()
		{
			struct addrinfo* result = NULL, * ptr = NULL, hints;

			const int recvbuflen = 2048;
			char recvbuf[recvbuflen];
			memset(recvbuf, 0, sizeof(recvbuf));

			int clientInfo;
			int recvResult;

			// 初始化 Winsock
			WSADATA wsaData;
			clientInfo = WSAStartup(MAKEWORD(2, 2), &wsaData);
			if (clientInfo != 0) {
				printf("WSAStartup failed with error: %d\n", clientInfo);
				return;
			}

			ZeroMemory(&hints, sizeof(hints));
			hints.ai_family = AF_UNSPEC;
			hints.ai_socktype = SOCK_STREAM;
			hints.ai_protocol = IPPROTO_TCP;

			// 解析服务器地址和端口
			clientInfo = getaddrinfo(tcpServerIp.c_str(), tcpServerPort.c_str(), &hints, &result);
			if (clientInfo != 0) {
				printf("getaddrinfo failed with error: %d\n", clientInfo);
				WSACleanup();
				return;
			}

			// 尝试连接地址直到成功
			for (ptr = result; ptr != NULL; ptr = ptr->ai_next) {
				// 创建就和server的sorcket连接
				ConnectSocket = socket(ptr->ai_family, ptr->ai_socktype, ptr->ai_protocol);
				if (ConnectSocket == INVALID_SOCKET) {
					printf("socket failed with error: %ld\n", WSAGetLastError());
					WSACleanup();
					return;
				}

				// 连接服务器.
				clientInfo = connect(ConnectSocket, ptr->ai_addr, (int)ptr->ai_addrlen);
				if (clientInfo == SOCKET_ERROR) {
					closesocket(ConnectSocket);
					ConnectSocket = INVALID_SOCKET;
					continue;
				}
				break;
			}

			freeaddrinfo(result);

			if (ConnectSocket == INVALID_SOCKET) {
				printf("Unable to connect to server!\n");
				WSACleanup();
				return;
			}

			unsigned char value[4] = { 0 };
			float tcps[6] = { 0 };
			float joints[6] = { 0 };

			while (true)
			{
				/*接受数据*/
				recvResult = recv(ConnectSocket, recvbuf, recvbuflen, 0);

				if (recvResult > 0)
				{
					Parse(recvbuf, tcps, joints, value);
				}
			};

			// cleanup
			closesocket(ConnectSocket);
			WSACleanup();
		}

		void Parse(char* recvbuf, float* tcps, float* joints, unsigned char* value) {
			EnumRobState robState = static_cast<EnumRobState>(recvbuf[1449]);// 机器人状态
			EnumProgramState runState = static_cast<EnumProgramState>(recvbuf[1450]); // 运动状态
			EnumSafetyState safetyState = static_cast<EnumSafetyState>(recvbuf[1451]); // 安全状态

			int clashSign = recvbuf[1452]; // 碰撞信号
			int clashArm = recvbuf[1453]; // 碰撞轴    

			// 机器人错误代码
			// ErrorCode = BitConverter.ToUInt32(buffer, 1456);

			/*if (safetyState != EnumSafetyState::SS_RUN) {
				LOG(INFO) << "机械臂安全异常";
			}*/

			// 错误代码
			value[0] = recvbuf[1456];
			value[1] = recvbuf[1456 + 1];
			value[2] = recvbuf[1456 + 2];
			value[3] = recvbuf[1456 + 3];
			int errCode = recvbuf[1456];
			memcpy(&errCode, value, 4);

			// 关节角度
			for (int i = 0; i < 6; i++) {
				value[0] = recvbuf[4 * i];
				value[1] = recvbuf[4 * i + 1];
				value[2] = recvbuf[4 * i + 2];
				value[3] = recvbuf[4 * i + 3];
				memcpy(&joints[i], value, 4);
			}
			// tcp
			for (int i = 0; i < 6; i++)
			{
				value[0] = recvbuf[4 * i + 368];
				value[1] = recvbuf[4 * i + 368 + 1];
				value[2] = recvbuf[4 * i + 368 + 2];
				value[3] = recvbuf[4 * i + 368 + 3];
				memcpy(&tcps[i], value, 4);
			}

			lastPushTsMs = Utils::Utils::GetTsMs();

			// 设置2为新数组
			if (newArrIdx == 1) {
				for (int i = 0; i < 6; i++) {
					joints2[i] = joints[i];
					tcps2[i] = tcps[i];
				}
				newArrIdx = 2;
			}
			else {// 设置1为新数组
				for (int i = 0; i < 6; i++) {
					joints1[i] = joints[i];
					tcps1[i] = tcps[i];
				}
				newArrIdx = 1;
			}
			memset(recvbuf, 0, sizeof(recvbuf));          //清空数组

			// 转换未vector
			std::vector<float> vjoints, vtcps;
			for (int i = 0; i < 6; i++) {
				vjoints.push_back(joints[i]);
			}
			for (int i = 0; i < 6; i++) {
				vtcps.push_back(tcps[i]);
			}

			mainthread::Proxy::UpdateRobState(robId, robState, runState, vjoints, vtcps, safetyState,errCode,clashSign,clashArm);
		}
	};
}