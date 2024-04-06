#pragma once
#include "arm_def.h"
#include "PIDmethod.h"
#include "Upper_Public.h"

//结合机械臂运动学与实际电机关节控制
template<uint8_t jNum>
class Arm_Controller_s
{
public:
	float timeStep = 0;
	PIDmethod Motor_pid[jNum];//关节pid控制器
	myMatrices<float> target_angle = myMatrices<float>(1, jNum);//目标关节角度
	myMatrices<float> current_angle = myMatrices<float>(1, jNum);//当前关节角度
	myMatrices<float> targetSpace = myMatrices<float>(4, 4);//目标齐次矩阵
	myMatrices<float> oriWorkSpace = myMatrices<float>(4, 4);//默认位型复位
	bool is_reset = false;//复位标志位
//public:
	Arm_Controller_s(float _timeStep,rOrder _order) :timeStep(_timeStep)
	{
		for (int i = 0; i < jNum; i++)
		{
			Motor_pid[i].PID_Init(Common, timeStep);
		}
		arm_dof.set_eularOrder(_order);
	}
	void PID_params(uint8_t _num, float _p, float _i, float _d, float _imax, float _omax, float _omin);
	Arm_Def_s<jNum> arm_dof;
	void setOriginAngle(const float _oangle[jNum]);//设置初始关节位型，赋予目标值和当前值
	void updateAngle_c(const float _cangle[jNum]);//更新观测值
	void updatePos_t(float _droll, float _dpitch, float _dyaw, float _dx, float _dy, float _dz);//四个算子
	void actuate();//控制器执行
	void setReset(bool _reset)
	{
		is_reset = _reset;
	}
	float motor_out[jNum];
};

template<uint8_t jNum>
void Arm_Controller_s<jNum>::PID_params(uint8_t _num, float _p, float _i, float _d, float _imax, float _omax, float _omin)
{
	Motor_pid[_num].Params_Config(_p, _i, _d, _imax, _omax, _omin);
}

template<uint8_t jNum>
void Arm_Controller_s<jNum>::setOriginAngle(const float _oangle[jNum])
{
	arm_dof.update_Space_c(_oangle);
	//arm_dof.update_Space_t(arm_dof.workArray_c.getArray());//定义初始位型
	arm_dof.update_Space_t(arm_dof.workSpace_c);
	target_angle = arm_dof.jointSpace_t;
	oriWorkSpace = arm_dof.workSpace_t;
	arm_dof.workArray_c.print();
	arm_dof.jointSpace_t.print();
}

template<uint8_t jNum>
void Arm_Controller_s<jNum>::updateAngle_c(const float _cangle[jNum])
{
	current_angle.setArray(_cangle, jNum);
	arm_dof.update_Space_c(_cangle);
}

template<uint8_t jNum>
void Arm_Controller_s<jNum>::updatePos_t(float _droll, float _dpitch, float _dyaw, float _dx, float _dy,float _dz)
{
	myMatrices<float> Tpitch(4);
	myMatrices<float> Tyaw(4);
	myMatrices<float> Troll(4);
	myMatrices<float> Tx(4);
	myMatrices<float> Ty(4);
	myMatrices<float> Tz(4);
	const float Apitch[4 * 4] = { cosf(_dpitch), 0,sinf(_dpitch),0,
											  0, 1,			   0,0,
								 -sinf(_dpitch), 0,cosf(_dpitch),0,
											  0, 0,			   0,1 };
	const float Ayaw[4 * 4] = { cosf(_dyaw),-sinf(_dyaw),0, 0,
								sinf(_dyaw), cosf(_dyaw),0, 0,
										  0,		   0,1, 0,
										  0,		   0,0, 1 };
	const float Aroll[4 * 4] = { 1,			0,			    0, 0,
								 0,cosf(_droll),-sinf(_droll), 0,
								 0,sinf(_droll), cosf(_droll), 0,
								 0,			0,			    0, 1 };
	const float Ax[4 * 4] = { 1,0,0,_dx,
							  0,1,0,  0,
							  0,0,1,  0,
							  0,0,0,  1 };
	const float Ay[4 * 4] = { 1,0,0,  0,
							  0,1,0,_dy,
							  0,0,1,  0,
							  0,0,0,  1 };
	const float Az[4 * 4] = { 1,0,0,  0,
							  0,1,0,  0,
							  0,0,1,_dz,
							  0,0,0,  1 };
	Tpitch.setArray(Apitch, 4 * 4);
	Tyaw.setArray(Ayaw, 4 * 4);
	Troll.setArray(Aroll, 4 * 4);
	Tx.setArray(Ax, 4 * 4);
	Ty.setArray(Ay, 4 * 4);
	Tz.setArray(Az, 4 * 4);
	myMatrices<float> workSpace_t = arm_dof.workSpace_t;
	workSpace_t = workSpace_t * Tpitch;
	workSpace_t = workSpace_t * Tyaw;
	workSpace_t = workSpace_t * Troll;
	workSpace_t = workSpace_t * Tx;
	workSpace_t = workSpace_t * Ty;
	workSpace_t = workSpace_t * Tz;
	if (is_reset)
	{
		workSpace_t = oriWorkSpace;
		workSpace_t.print();
	}
	arm_dof.update_Space_t(workSpace_t);
	target_angle = arm_dof.jointSpace_t;
}

template<uint8_t jNum>
void Arm_Controller_s<jNum>::actuate()
{
	for (int i = 0; i < jNum; i++)
	{
		Motor_pid[i].target = target_angle.getElement(0, i);
		Motor_pid[i].current = current_angle.getElement(0, i);
		motor_out[i] = Motor_pid[i].Adjust(0);
	}
}
