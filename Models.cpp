#pragma once
#include <string>
#include <vector>
#include <json.hpp>
#include "Utils/base64/base64.h"

using json = nlohmann::json;

namespace RobHelper {
	/**
	 * @������:����ǿ
	 * @��������:2023-10-23
	 * @����: ָ��
	 * **/
	struct Cmd
	{
		// ָ������,����(moveJ moveL  moveJpose moveTcp sleep)
		std::string name;

		// ָ�����(�ǶȻ�ȡλ��,����Ϊ6)
		std::vector<float> params;

		// ��������ϵ����
		std::string tool = "";

		// ��������ϵ����
		std::string workpiece = "";
	};

	/**
	 * @������:����ǿ
	 * @��������:2023-10-23
	 * @����:lua�ű�ģ��
	 * **/
	class LuaModel
	{
	public:
		int robId; // ������ID
		int batchNo; // ���κ�
		std::string	type; // ָ������
		int param; // ָ��
		float paramf = 0.0;// �������,���������Ĳ�ͬ����
		std::string luaContent; // lua�ű�����

		/**
		 * @������:����ǿ
		 * @����:ת��Ϊjson�ַ���
		 * @����ֵ:json�ַ���
		 * **/
		std::string toJson() {
			json body;
			body["RobId"] = robId;
			body["batchNo"] = batchNo;
			body["type"] = type;
			body["param"] = param;
			body["paramf"] = paramf;
			body["style"] = "lua";
			body["params"] = base64_encode(luaContent, false);
			return body.dump();
		}
	};

	class JsonModel
	{
	private:
		std::string style = "json"; // ָ������Ϊjson����
	public:
		int robId; // ������ID
		int batchNo; // ���κ�
		std::string	type; // ָ������
		int param = 0;	 // ����ָ��ֵ
		float paramf = 0.0;// �������,���������Ĳ�ͬ����
		std::vector<Cmd> params; // ָ������

		/**
		 * @������:����ǿ
		 * @����:ת��Ϊjson�ַ���
		 * @����ֵ:json�ַ���
		 * **/
		std::string toJson() {
			json body;
			body["RobId"] = robId;
			body["batchNo"] = batchNo;
			body["type"] = type;
			body["param"] = param;
			body["paramf"] = paramf;
			body["style"] = "json";

			std::vector<nlohmann::json_abi_v3_11_2::json> cmds;
			for (const Cmd& c : params) {
				json cmd;
				cmd["name"] = c.name;
				cmd["tool"] = c.tool;
				cmd["workpiece"] = c.workpiece;
				cmd["params"] = c.params;
				cmds.push_back(cmd);
			}
			body["params"] = cmds;

			return body.dump();
		}
	};

	/**
	 * @������:����ǿ
	 * @��������:2023-10-25
	 * @����:������״̬ö��
	 * **/
	enum class EnumRobState
	{
		SR_Start = 0, //����������
		SR_Initialize = 1, //�����˳�ʼ��
		SR_Logout = 2, //�������˳���½����δʹ��
		SR_Login = 3, //�����˵�½����δʹ��
		SR_PowerOff = 4, //�����µ�
		SR_Disable = 5, //������ʧ��
		SR_Enable = 6 //������ʹ��
	};

	/**
	 * @������:����ǿ
	 * @��������:2023-10-25
	 * @����:��������״̬ö��
	 * **/
	enum class EnumProgramState
	{
		SP_Stopped = 0, //����ֹͣ
		SP_Stopping = 1, //��������ֹͣ��
		SP_Running = 2, //������������
		SP_Paused = 3, //�����Ѿ���ͣ
		SP_Pausing = 4, //������ͣ��
		SP_TaskRuning = 5 //�ֶ�ʾ������ִ����
	};

	/**
	 * @������:����ǿ
	 * @��������:2023-10-18
	 * @����:˫��״̬����ײ����
	 * **/
	struct RobState
	{
		// ���������Ƿ���ȷ
		bool isOk;

		// ��е��1��״̬
		EnumRobState rob1State = EnumRobState::SR_Start;

		// Nc��������״̬
		EnumProgramState rob1ProgramState = EnumProgramState::SP_Stopped;

		// �����˹ؽ�
		std::vector<float> rob1Joints;

		// ĩ������
		std::vector<float> rob1Tcps;

		// ��е��2��״̬
		EnumRobState rob2State = EnumRobState::SR_Start;

		// Nc��������״̬
		EnumProgramState rob2ProgramState = EnumProgramState::SP_Stopped;

		// �����˹ؽ�
		std::vector<float> rob2Joints;

		// ĩ������
		std::vector<float> rob2Tcps;

		// ��ײ����(��λmm,Ϊ0ʱ��˵���Ѿ���ײ)
		int clashDistanceMm = -1;
	};

	/**
	 * @������:����ǿ
	 * @��������:2023-10-24
	 * @����:������������Ϣ
	 * @����ֵ:
	 * **/
	class Msg
	{
	public:
		// status=0 ʧ�� ;  status=1 �ɹ�
		int status;

		// ��ʾ��Ϣ
		std::string msg;

		// ʵ����Ϣ��
		std::string data;

		/**
		 * @������:����ǿ
		 * @��������:2023-10-25
		 * @����:���캯��,Ĭ������Ϊʧ��
		 * **/
		Msg(int p_status = 0, std::string p_msg = "fail", std::string p_data = "") {
			status = p_status;
			msg = p_msg;
			data = p_data;
		}

		// �Ƿ�ɹ�
		bool IsOk() {
			return status == 1;
		}
	};
}