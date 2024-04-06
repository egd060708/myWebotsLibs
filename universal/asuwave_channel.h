#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <stack>
#include <json/json.h>

class Asuwave_Channel {
private:
	uint32_t timeStep;//采样周期
	uint32_t timeStamp = 0;//时间戳
	std::string name[9];//变量名
	double data[9];//变量值
	std::string uint2string(uint32_t _temp);

public:
	Asuwave_Channel():timeStep(5){}//构造函数，默认采样周期5ms
	void updateTime(uint32_t _timeStep);//更新采样周期
	void packData(const double _data[9]);//变量打包，输入变量名称和变量的值
	void writeJson(const std::string _Filepath, std::ios_base::openmode _Mode = std::ios_base::out);//导出文件

};

void Asuwave_Channel::updateTime(uint32_t _timeStep)
{
	timeStep = _timeStep;
}

void Asuwave_Channel::packData(const double _data[9])
{
	//定义通道名称,自行修改
	name[0] = "channel_1";
	name[1] = "channel_2";
	name[2] = "channel_3";
	name[3] = "channel_4";
	name[4] = "channel_5";
	name[5] = "channel_6";
	name[6] = "channel_7";
	name[7] = "channel_8";
	name[8] = "channel_9";
	//数据拷贝
	for (int i = 0; i < 9; i++)
	{
		data[i] = _data[i];
	}
}

void Asuwave_Channel::writeJson(const std::string _Filepath, std::ios_base::openmode _Mode)
{
	timeStamp += timeStep;
	std::string timeStr = uint2string(timeStamp);//把时间戳转换成字符串

	Json::Value root;//根节点
	Json::Value friends;//子节点
	for (int i = 0; i < 9; i++)
	{
		friends[name[i]] = Json::Value(data[i]);
	}
	root[timeStr] = Json::Value(friends);

	//输出到文件 
	Json::StyledWriter sw;
	std::ofstream os;
	os.open(_Filepath + "\\webots.json", _Mode);//app写入时追加，trunc写入时清除旧数据
	if (!os.is_open())
		std::cout << "error：can not find or create the file which named \" demo.json\"." << std::endl;
	os << sw.write(root);
	os.close();
}

std::string Asuwave_Channel::uint2string(uint32_t _temp)
{
	if (_temp == 0)
		return "0";

	//数字入栈
	std::stack<char> stack;
	while (_temp > 0) {
		stack.push((_temp % 10) + '0');
		_temp = _temp / 10;
	}
	//数字出栈
	std::string result = "";
	while (!stack.empty()) {
		result += stack.top();
		stack.pop();
	}
	return result;
}
