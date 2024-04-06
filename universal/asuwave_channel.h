#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <stack>
#include <json/json.h>

class Asuwave_Channel {
private:
	uint32_t timeStep;//��������
	uint32_t timeStamp = 0;//ʱ���
	std::string name[9];//������
	double data[9];//����ֵ
	std::string uint2string(uint32_t _temp);

public:
	Asuwave_Channel():timeStep(5){}//���캯����Ĭ�ϲ�������5ms
	void updateTime(uint32_t _timeStep);//���²�������
	void packData(const double _data[9]);//�������������������ƺͱ�����ֵ
	void writeJson(const std::string _Filepath, std::ios_base::openmode _Mode = std::ios_base::out);//�����ļ�

};

void Asuwave_Channel::updateTime(uint32_t _timeStep)
{
	timeStep = _timeStep;
}

void Asuwave_Channel::packData(const double _data[9])
{
	//����ͨ������,�����޸�
	name[0] = "channel_1";
	name[1] = "channel_2";
	name[2] = "channel_3";
	name[3] = "channel_4";
	name[4] = "channel_5";
	name[5] = "channel_6";
	name[6] = "channel_7";
	name[7] = "channel_8";
	name[8] = "channel_9";
	//���ݿ���
	for (int i = 0; i < 9; i++)
	{
		data[i] = _data[i];
	}
}

void Asuwave_Channel::writeJson(const std::string _Filepath, std::ios_base::openmode _Mode)
{
	timeStamp += timeStep;
	std::string timeStr = uint2string(timeStamp);//��ʱ���ת�����ַ���

	Json::Value root;//���ڵ�
	Json::Value friends;//�ӽڵ�
	for (int i = 0; i < 9; i++)
	{
		friends[name[i]] = Json::Value(data[i]);
	}
	root[timeStr] = Json::Value(friends);

	//������ļ� 
	Json::StyledWriter sw;
	std::ofstream os;
	os.open(_Filepath + "\\webots.json", _Mode);//appд��ʱ׷�ӣ�truncд��ʱ���������
	if (!os.is_open())
		std::cout << "error��can not find or create the file which named \" demo.json\"." << std::endl;
	os << sw.write(root);
	os.close();
}

std::string Asuwave_Channel::uint2string(uint32_t _temp)
{
	if (_temp == 0)
		return "0";

	//������ջ
	std::stack<char> stack;
	while (_temp > 0) {
		stack.push((_temp % 10) + '0');
		_temp = _temp / 10;
	}
	//���ֳ�ջ
	std::string result = "";
	while (!stack.empty()) {
		result += stack.top();
		stack.pop();
	}
	return result;
}
