/*! @file manipulator.h
 *  @brief Data structs for manipulator robots' params
 *
 *  This file contains the Manipulator_Classdef,which provide ways to store manipulator robots' params.
 *  此文件用于管理单腿参数
 */
#ifndef _MANIPULATOR_H_
#define _MANIPULATOR_H_

#define DEGREE_TO_RADIAN 3.14159/180.

#define RF_JOINT_OFFSET -20.
#define RB_JOINT_OFFSET 200.
#define LF_JOINT_OFFSET -20.
#define LB_JOINT_OFFSET 200.
#define F_JOINT_MAX 80.
#define F_JOINT_MIN -30.
#define B_JOINT_MAX 210.
#define B_JOINT_MIN 100.

 //连杆长度
struct Bipe_Structdef
{
    float L1;
    float L2;
    float L3;
    float L4;
    float L5;
};

//等效摆杆
struct Pendulum_Structdef
{
    float mass;         //重量（质心）
    float length;       //杆长
    float last_length;  //上一次杆长
    float dlength;      //杆长微分
    float angle = 0;        //摆角
    float last_angle;   //上一次角度
    float dangle;       //角度微分
    float k;            //重心长度系数length*(1-k)
};

//等效关节
struct Joint_Structdef
{
    float f_angle;
    float b_angle;
    float position_x;
    float position_y;
    float last_y;
    float dposition_y;
    Pendulum_Structdef pendulum;
    float Fx;
    float Fy;
    float f_torque;
    float b_torque;
};

//力矩
struct Torque_Structdef
{
    float wheel;
    float f_joint;
    float b_joint;
};

/*轮腿类*/
class Manipulator_Classdef
{
public:
    Manipulator_Classdef(float _t = 0) { dt = _t; }
    void Init(float _f_offset, float _b_offset, float _f_max, float _f_min, float _b_max, float _b_min); //初始化机械参数

    void current_joint_update(float _f_angel, float _b_angel);//更新关节角度
    void body_angle_update(float _angle,float _dangle);   //更新车身角度
    void timer_update(float _dt);           //更新时间微分

    void forward_kinematics_cal();//正运动学解算
    void forward_jacobian();//正雅可比VMC
    void overall_barycenter_cal();//重心计算

    Joint_Structdef target_joint;    //目标轮腿参数
    Joint_Structdef current_joint;   //当前轮腿参数
    Torque_Structdef torque_output;  //输出力矩参数
    Bipe_Structdef bipes;            //五连杆参数
    Pendulum_Structdef body;         //车身摆杆等效参数
    Pendulum_Structdef overall;      //车身与轮腿拟合参数

private:
    //State_Data_Classdef* state;
    float dt;
    float f_joint_offset;   //前关节初始角度
    float b_joint_offset;   //后关节初始角度
    float f_joint_max;      //前关节最大角度
    float b_joint_max;      //后关节最大角度
    float f_joint_min;      //前关节最小角度
    float b_joint_min;      //后关节最小角度
};


//初始化参数
void Manipulator_Classdef::Init(float _f_offset, float _b_offset, float _f_max, float _f_min, float _b_max, float _b_min)
{
    bipes.L1 = 0.15;
    bipes.L2 = 0.288;
    bipes.L3 = 0.288;
    bipes.L4 = 0.15;
    bipes.L5 = 0.15;

    body.mass = 15.071;
    body.length = 0.024162;
    body.k = 0;

    target_joint.pendulum.length = 0.14;
    target_joint.pendulum.last_length = 0.14;

    current_joint.pendulum.mass = 0.784;
    current_joint.pendulum.k = 0.4037;

    f_joint_offset = _f_offset * DEGREE_TO_RADIAN;
    b_joint_offset = _b_offset * DEGREE_TO_RADIAN;
    f_joint_max = _f_max * DEGREE_TO_RADIAN;
    b_joint_max = _b_max * DEGREE_TO_RADIAN;
    f_joint_min = _f_min * DEGREE_TO_RADIAN;
    b_joint_min = _b_min * DEGREE_TO_RADIAN;
}

/**
 * @brief  五连杆正向运动学
 * @note
 * @param
 * @return
 * @retval  None
 */
void Manipulator_Classdef::forward_kinematics_cal()
{
    //body.angle = state->current_pos.pitch;
    float X3 = bipes.L5 / 2. + bipes.L1 * cos(current_joint.f_angle);
    float Y3 = -bipes.L1 * sin(current_joint.f_angle);
    float X4 = -bipes.L5 / 2. + bipes.L4 * cos(current_joint.b_angle);
    float Y4 = -bipes.L4 * sin(current_joint.b_angle);
    float X43 = X3 - X4;
    float Y43 = Y3 - Y4;
    float X6 = (X3 + X4) / 2.;
    float Y6 = (Y3 + Y4) / 2.;
    float D43 = sqrt(pow(X43, 2) + pow(Y43, 2));
    current_joint.position_x = sqrt((pow(bipes.L2, 2) - pow(D43, 2) / 4.) / pow(D43, 2)) * Y43 + X6;
    current_joint.position_y = sqrt((pow(bipes.L3, 2) - pow(D43, 2) / 4.) / pow(D43, 2)) * (-X43) + Y6;
    if (dt > 0) {
        current_joint.dposition_y = (current_joint.position_y - current_joint.last_y) / dt;//y方向的变化率
    }
    current_joint.last_y = current_joint.position_y;

    current_joint.pendulum.length = sqrt(pow(current_joint.position_x, 2) + pow(current_joint.position_y, 2));
    if (dt > 0) {
        current_joint.pendulum.dlength = (current_joint.pendulum.length - current_joint.pendulum.last_length) / dt;//杆长变化率
    }
    current_joint.pendulum.last_length = current_joint.pendulum.length;

    current_joint.pendulum.angle = atan(current_joint.position_x / current_joint.position_y) + body.angle;
    if (dt > 0) {
        current_joint.pendulum.dangle = (current_joint.pendulum.angle - current_joint.pendulum.last_angle) / dt + body.dangle;//求角速度
    }
    current_joint.pendulum.last_angle = current_joint.pendulum.angle;
}

/**
 * @brief  五连杆正向雅可比（VMC）
 * @note
 * @param
 * @return 两个电机力矩
 * @retval  None
 */
void Manipulator_Classdef::forward_jacobian()
{
    float A = pow(bipes.L1, 2) * pow((sin(current_joint.b_angle) - sin(current_joint.f_angle)), 2) + pow(bipes.L5 + bipes.L1 * (cos(current_joint.f_angle) - cos(current_joint.b_angle)), 2);
    float B = -2. * pow(bipes.L1, 2) * (sin(current_joint.b_angle) - sin(current_joint.f_angle)) * cos(current_joint.f_angle) - 2. * (bipes.L5 + bipes.L1 * (cos(current_joint.f_angle) - cos(current_joint.b_angle))) * bipes.L1 * sin(current_joint.f_angle);
    float C = 2. * pow(bipes.L1, 2) * (sin(current_joint.b_angle) - sin(current_joint.f_angle)) * cos(current_joint.b_angle) + 2. * (bipes.L5 + bipes.L1 * (cos(current_joint.f_angle) - cos(current_joint.b_angle))) * bipes.L1 * sin(current_joint.b_angle);
    float D = sqrt(pow(bipes.L2, 2) / A - 0.25);
    float E = (bipes.L1 * (sin(current_joint.f_angle) - sin(current_joint.b_angle)) * pow(bipes.L2, 2)) / (2. * pow(A, 2) * D);
    float F = ((bipes.L5 + bipes.L1 * (cos(current_joint.f_angle) - cos(current_joint.b_angle))) * pow(bipes.L2, 2)) / (2 * pow(A, 2) * D);

    float x_a1 = E * B - bipes.L1 * cos(current_joint.f_angle) * D - 0.5 * bipes.L1 * sin(current_joint.f_angle);
    float x_a2 = E * C + bipes.L1 * cos(current_joint.b_angle) * D - 0.5 * bipes.L1 * sin(current_joint.b_angle);
    float y_a1 = F * B + bipes.L1 * sin(current_joint.f_angle) * D - 0.5 * bipes.L1 * cos(current_joint.f_angle);
    float y_a2 = F * C - bipes.L1 * sin(current_joint.b_angle) * D - 0.5 * bipes.L1 * cos(current_joint.b_angle);

    current_joint.f_torque = (x_a1 * current_joint.Fx + y_a1 * current_joint.Fy);//符号根据建模和matlab结果综合整定
    current_joint.b_torque = (x_a2 * current_joint.Fx + y_a2 * current_joint.Fy);
}

/**
 * @brief  整车质心解算(仅对单腿)
 * @note    运用惯性矩原理
 * @param
 * @return
 * @retval  None
 */
void Manipulator_Classdef::overall_barycenter_cal()
{
    float X1 = current_joint.pendulum.length * (1. - current_joint.pendulum.k) * sin(current_joint.pendulum.angle);
    float Y1 = current_joint.pendulum.length * (1. - current_joint.pendulum.k) * cos(current_joint.pendulum.angle);
    float X2 = current_joint.pendulum.length * sin(current_joint.pendulum.angle) + body.length * (1 - body.k) * sin(body.angle);
    float Y2 = current_joint.pendulum.length * cos(current_joint.pendulum.angle) + body.length * (1 - body.k) * cos(body.angle);
    float X12 = X2 - X1;
    float Y12 = Y2 - Y1;
    float D12 = sqrt(pow(X12, 2) + pow(Y12, 2));
    float Angle12 = atan(X12 / Y12);
    float k3 = current_joint.pendulum.mass / (body.mass / 2.f) / (1 + current_joint.pendulum.mass / (body.mass / 2.));//这里相当于取一半的车体质量
    float X3 = X1 + D12 * (1 - k3) * sin(Angle12);
    float Y3 = Y1 + D12 * (1 - k3) * cos(Angle12);
    overall.mass = current_joint.pendulum.mass + body.mass / 2.;
    overall.length = sqrt(pow(X3, 2) + pow(Y3, 2));
    overall.angle = atan(X3 / Y3);
    overall.k = 0;
}


void Manipulator_Classdef::current_joint_update(float _f_angel, float _b_angel)
{
    current_joint.f_angle = _f_angel + f_joint_offset;
    current_joint.b_angle = _b_angel + b_joint_offset;
}


void Manipulator_Classdef::body_angle_update(float _angle,float _dangle)
{
    body.angle = _angle;
    body.dangle = _dangle;
}

void Manipulator_Classdef::timer_update(float _dt)
{
    dt = _dt;
}

#endif
