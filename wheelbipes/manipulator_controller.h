/*! @file manipulator_controller.h
 *  @brief Controll methods of manipulator robots
 *
 *  This file contains the Manipulator_Controller_Classdef,
 *  which provide all controllers to controll manipulator robots.
 *  轮腿控制器
 */
#ifndef _MANIPULATOR_CONTROLLER_H_
#define _MANIPULATOR_CONTROLLER_H_

#include "lqrCalculater.h"
#include "manipulator.h"
#include "user_data.h"
#include "Upper_Public.h"
#include "state_data.h"
#include "kalman_filter.h"

USING_NAMESPACE_MM
USING_NAMESPACE_QPOASES

#define MPC_CALC MPC_CalculatorClassdef<6, 3, 1, 5>
#define KALMAN_CALC kalmanFilter<real_t,6, 3>

/*轮控制器参数列表*/
enum wheelCtrl_Enumdef
{
    W_DISTANCE,
    W_SPEED,
    W_ALPHA,
    W_BETA,
    W_FF,
    W_TURN
};
/*腿控制器参数列表*/
enum jointCtrl_Enumdef
{
    J_DISTANCE,
    J_SPEED,
    J_ALPHA,
    J_BETA,
    J_TURN,
    J_LENGTHKEEP
};

enum legside_Enumdef
{
    RIGHT_JOINT,
    LEFT_JOINT
};

#ifdef __cplusplus
extern "C"
{
#endif
    class Manipulator_Controller_Classdef
    {
    public:
        Manipulator_Controller_Classdef() {}
        Manipulator_Controller_Classdef(State_Data_Classdef* _sd, Manipulator_Classdef* r_mp, Manipulator_Classdef* l_mp, UserData_Classdef* _user)
        {
            state = _sd;
            user = _user;
            mp[RIGHT_JOINT] = r_mp;
            mp[LEFT_JOINT] = l_mp;
        }
        //参数初始化
        void Init();
        //引用类型加载
        void Load_Reference_Type(State_Data_Classdef* _sd, Manipulator_Classdef* r_mp, Manipulator_Classdef* l_mp, UserData_Classdef* _user);
        //控制器加载
        void Load_Lqr_Controller(lqrCalculater<6, 2>* _lqrCal);
        void Load_Mpc_Controller(MPC_CALC* _mpcCal,modelFit<6,6,3>* _A,modelFit<6,3,3>* _B);
        void Load_Wheel_SubController(PIDmethod* _wSubCtrl);
        void Load_Joint_SubController(PIDmethod* _jSubCtrl);
        //控制器使能
        void Set_Enable_List(bool* _w, bool* _j);
        //控制结算
        void controll_adjust();

    private:
        State_Data_Classdef* state;
        UserData_Classdef* user;
        Manipulator_Classdef* mp[2];

        /*功能动作函数*/
        void jump_adjust();

        /*LQR闭环增益*/
        void lqr_adjust(Manipulator_Classdef* _mp, lqrCalculater<6, 2>* _lqr_cal);
        void lqr_state_config(Manipulator_Classdef* mp, float* state_target, float* state_current);

        /*mpc控制*/
        void mpc_adjust(Manipulator_Classdef* _mp, MPC_CALC* _mpcCal, modelFit<6, 6, 3>* _A, modelFit<6, 3, 3>* _B);
        void mpc_adjust_kalman(Manipulator_Classdef* _mp, MPC_CALC* _mpcCal, modelFit<6, 6, 3>* _A, modelFit<6, 3, 3>* _B, KALMAN_CALC* _kalmanF);
        void mpc_adjust(Manipulator_Classdef* _mp, MPC_CALC* _mpcCal, modelFit<6, 6, 3>* _A, modelFit<6, 3, 3>* _B, real_t ext_uk[3]);
        void mpc_state_config(Manipulator_Classdef* mp, real_t* state_target, real_t* state_current);


        float wheel_turn_adjust();                                 //转向环
        float wheel_feedforward_adjust(Manipulator_Classdef* _mp); //前馈环

        float length_keep_adjust(Manipulator_Classdef* _mp);                                      //轮腿腿长控制
        float joint_turn_adjust(Manipulator_Classdef* _mp_right, Manipulator_Classdef* _mp_left); //转向防劈叉
        float roll_keep_adjust();                                                                 //车体roll轴平衡
        float turn_adaption_adjust();                                                             //车体运动转向roll内倾
        float turn_adaption_adjust2();                                                            //车体运动转向力矩补偿2
        float turn_adaption_adjust3();                                                            //车体运动转向力矩补偿3

        /****************************控制器********************************/
        lqrCalculater<6, 2>* lqrCal[2]; //腿的极性和轮的极性是相反的
        MPC_CALC* mpcCal[2];
        modelFit<6, 6, 3>* model_A;//状态矩阵
        modelFit<6, 3, 3>* model_B;//输出矩阵

        PIDmethod* w_turn_pid;
        PIDmethod* j_turn_pid;

        PIDmethod* j_follow_pid;
        PIDmethod* j_roll_keep_pid;
        PIDmethod* j_length_keep_pid;

        /*创建状态变量数组*/
        float lqr_target[6];
        float lqr_current[6];

        real_t mpc_target[6];
        real_t mpc_current[6];
        /*控制器使能数组*/
        bool wheel_enable_list[6] = { 0 };
        bool joint_enable_list[6] = { 0 };

        /****************************滤波器********************************/
        KALMAN_CALC kalmanMpc[2];//定义一个六状态六输入的卡尔曼滤波器

        MATRIX out = MATRIX(3, 1);
    };
#ifdef __cplusplus
};
#endif

/****************************加载外部引用类型********************************/
void Manipulator_Controller_Classdef::Load_Reference_Type(State_Data_Classdef* _sd, Manipulator_Classdef* r_mp, Manipulator_Classdef* l_mp, UserData_Classdef* _user)
{
    state = _sd;
    user = _user;
    mp[RIGHT_JOINT] = r_mp;
    mp[LEFT_JOINT] = l_mp;
}

/****************************加载控制器********************************/
void Manipulator_Controller_Classdef::Load_Lqr_Controller(lqrCalculater<6, 2>* _lqrCal)
{
    lqrCal[RIGHT_JOINT] = &_lqrCal[RIGHT_JOINT];
    lqrCal[LEFT_JOINT] = &_lqrCal[LEFT_JOINT];
}

void Manipulator_Controller_Classdef::Load_Mpc_Controller(MPC_CALC* _mpcCal, modelFit<6, 6, 3>* _A, modelFit<6, 3, 3>* _B)
{
    mpcCal[RIGHT_JOINT] = &_mpcCal[RIGHT_JOINT];
    mpcCal[LEFT_JOINT] = &_mpcCal[LEFT_JOINT];
    model_A = _A;
    model_B = _B;
}

void Manipulator_Controller_Classdef::Load_Wheel_SubController(PIDmethod* _wSubCtrl)
{
    w_turn_pid = &_wSubCtrl[0];
    w_turn_pid->PID_Init(Common, state->dt);
}

void Manipulator_Controller_Classdef::Load_Joint_SubController(PIDmethod* _jSubCtrl)
{
    j_turn_pid = &_jSubCtrl[0];
    j_follow_pid = &_jSubCtrl[1];
    j_roll_keep_pid = &_jSubCtrl[2];
    j_length_keep_pid = &_jSubCtrl[3];
    j_turn_pid->PID_Init(Common, state->dt);
    j_follow_pid->PID_Init(Common, state->dt);
    j_roll_keep_pid->PID_Init(Common, state->dt);
    j_length_keep_pid->PID_Init(Common, state->dt);
}

void Manipulator_Controller_Classdef::Init()
{
    /*数据写入控制器*/
    lqrCal[0]->init(user->leg_lqr_params);
    lqrCal[1]->init(user->leg_lqr_params);
    w_turn_pid->Params_Config(user->lqr_yaw_kp, 0, numeric_limits<float>::max());
    j_roll_keep_pid->Params_Config(PID_Mode::IS_PD, user->roll_keep_kp, user->roll_keep_kd, 0, numeric_limits<float>::max());
    j_length_keep_pid->Params_Config(PID_Mode::IS_PD, user->length_keep_kp, user->length_keep_kd, 0, numeric_limits<float>::max());
    j_turn_pid->Params_Config(user->j_turn_kp, 0, numeric_limits<float>::max());
    j_turn_pid->d_of_current = false;
    j_follow_pid->Params_Config(PID_Mode::IS_PD, user->j_follow_kp, user->j_follow_kd, 0, numeric_limits<float>::max());
    j_follow_pid->d_of_current = false;
    for (int i = 0; i < 6; i++)//初始化使能列表为false
    {
        wheel_enable_list[i] = false;
        joint_enable_list[i] = false;
    }
    // 向模型类输入参数矩阵
    model_A->setFunctions(user->model_A);
    model_B->setFunctions(user->model_B);
}

/**
 * @brief  控制器使能函数
 * @note
 * @param   轮控制使能清单；腿控制使能清单；
 * @return
 * @retval  None
 */
void Manipulator_Controller_Classdef::Set_Enable_List(bool* _w, bool* _j)
{
    for (int i = 0; i < 6; i++)
    {
        if (_w[i] != false && _w[i] != true) {
            wheel_enable_list[i] = false;
        }
        else {
            wheel_enable_list[i] = _w[i];
        }
        if (_j[i] != false && _j[i] != true) {
            joint_enable_list[i] = false;
        }
        else {
            joint_enable_list[i] = _j[i];
        }
    }
}

/**
 * @brief  总控函数
 * @note
 * @param
 * @return
 * @retval  None
 */
void Manipulator_Controller_Classdef::controll_adjust()
{
    float wheel_output[2][6];
    float joint_output[2][6];
    real_t wheel_mpc_out[2];
    real_t joint_mpc_out[2];
    
    for (int i = 0; i < 2; i++)
    {
        mp[i]->body_angle_update(state->current_pos.pitch,state->current_av.pitch); //向腿杆更新车体倾角
        mp[i]->forward_kinematics_cal();
        mp[i]->overall_barycenter_cal();
    }
    /* 状态检测 */
    state->sport_adaption();
        jump_adjust();
    state->tiny_weightlessness_check();
    if (state->flags.leg_length_step)
    {
        state->weightlessness_check(); //跳跃模式下收完腿才开启腾空检测
    }
    /*轮控制*/
    wheel_turn_adjust();
    for (int j = 0; j < 2; j++)
    {
        /*lqr解算*/
        lqr_adjust(mp[j], lqrCal[j]);

        /*轮子控制参数导出*/
        wheel_output[j][W_DISTANCE] = lqrCal[j]->single_out[0][lqrID::distance];
        wheel_output[j][W_SPEED] = lqrCal[j]->single_out[0][lqrID::speed];
        wheel_output[j][W_ALPHA] = lqrCal[j]->single_out[0][lqrID::alpha] + lqrCal[j]->single_out[0][lqrID::dalpha];
        wheel_output[j][W_BETA] = lqrCal[j]->single_out[0][lqrID::beta] + lqrCal[j]->single_out[0][lqrID::dbeta];
        wheel_output[j][W_FF] = wheel_feedforward_adjust(mp[j]);
        wheel_output[j][W_TURN] = w_turn_pid->out;
        if (state->flags.break_flag)
        {
            wheel_output[j][W_SPEED] = upper::constrain(wheel_output[j][W_SPEED], -16000.f * 4.6f / (13.0f / 20.0f * 16384.0f), 16000.f * 4.6f / (13.0f / 20.0f * 16384.0f));
        }
        else
        {
            wheel_output[j][W_SPEED] = upper::constrain(wheel_output[j][W_SPEED], -10000.f * 4.6f / (13.0f / 20.0f * 16384.0f), 10000.f * 4.6f / (13.0f / 20.0f * 16384.0f));
        }
        /*轮腿控制部分(v是轮杆方向，h是轮杆法向)*/
        if (state->flags.weightlessness) {
            if (mp[j]->target_joint.pendulum.length < 0.22f || mp[j]->target_joint.pendulum.length > 0.4f)
            {
                mp[j]->target_joint.pendulum.length = 0.26f;
            }
        }
        else {}
    }

    state->target_pos.roll = turn_adaption_adjust(); //第一类转向补偿
    float length_error = roll_keep_adjust();         // roll计算
    if (state->flags.weightlessness == false && state->flags.leg_length_step == true)
    {
        mp[RIGHT_JOINT]->target_joint.pendulum.length += length_error;
        mp[LEFT_JOINT]->target_joint.pendulum.length -= length_error;
    }
    float turn_fv = 0;
    // turn_fv = turn_adaption_adjust2();//第二类转向补偿
    // turn_fv = turn_adaption_adjust3();//第三类转向补偿
    float theta[2];
    float fv[2];
    float fh[2];
    /*转向+劈叉控制*/
    joint_turn_adjust(mp[RIGHT_JOINT], mp[LEFT_JOINT]);
    for (int k = 0; k < 2; k++)
    {
        //失重或弹跳只留直立环
        if (state->flags.weightlessness || (!state->flags.leg_length_step) || state->flags.tiny_weightlessness)
        {
            wheel_enable_list[W_SPEED] = false;
            wheel_enable_list[W_DISTANCE] = false;
            wheel_enable_list[W_TURN] = false;
            wheel_enable_list[W_BETA] = false;
            joint_enable_list[J_DISTANCE] = false;
            joint_enable_list[J_SPEED] = false;
            joint_enable_list[J_BETA] = false;
        }
        else
        {}
        ////real_t lqr_out[3] = { (real_t)lqrCal[k]->total_out[0],(real_t)lqrCal[k]->total_out[0],(real_t)lqrCal[k]->total_out[1] };
        ////mpc_adjust(mp[k], mpcCal[k], model_A, model_B, lqr_out);

        //mpc_adjust_kalman(mp[k], mpcCal[k], model_A, model_B, &kalmanMpc[k]);
        ////mpc_adjust(mp[k], mpcCal[k], model_A, model_B);
        //out = mpcCal[k]->getOutput();
        //wheel_mpc_out[k] = out.getElement(1, 0);
        //joint_mpc_out[k] = -out.getElement(2, 0) / mp[k]->current_joint.pendulum.length;
        /*获得腿杆控制量*/
        joint_output[k][J_DISTANCE] = -lqrCal[k]->single_out[1][lqrID::distance] / mp[k]->current_joint.pendulum.length;
        joint_output[k][J_SPEED] = -lqrCal[k]->single_out[1][lqrID::speed] / mp[k]->current_joint.pendulum.length;
        joint_output[k][J_ALPHA] = -(lqrCal[k]->single_out[1][lqrID::alpha] + lqrCal[k]->single_out[1][lqrID::dalpha]) / mp[k]->current_joint.pendulum.length;
        joint_output[k][J_BETA] = -(lqrCal[k]->single_out[1][lqrID::beta] + lqrCal[k]->single_out[1][lqrID::dbeta]) / mp[k]->current_joint.pendulum.length;
        joint_output[k][J_TURN] = j_follow_pid->out - j_turn_pid->out;
        joint_output[k][J_LENGTHKEEP] = length_keep_adjust(mp[k]);
        /*腿杆相对于地面的角度*/
        theta[k] = mp[k]->current_joint.pendulum.angle - state->current_pos.pitch;
        /*输出*/
        
        /*根据使能清单关闭相应的控制模块*/
        for (int n = 0; n < 6; n++)
        {
            if (wheel_enable_list[n] == false) {
                wheel_output[k][n] = 0;
            }
            if (joint_enable_list[n] == false) {
                joint_output[k][n] = 0;
            }
        }
        /*力矩集成输出*/
        if (k == RIGHT_JOINT)
        {
            fv[k] = joint_output[k][J_LENGTHKEEP] + turn_fv;
            fh[k] = joint_output[k][J_DISTANCE] + joint_output[k][J_SPEED] + joint_output[k][J_ALPHA] + joint_output[k][J_BETA] + joint_output[k][J_TURN];
            //fh[k] = joint_mpc_out[k] + joint_output[k][J_TURN];
        }
        else
        {
            fv[k] = joint_output[k][J_LENGTHKEEP] - turn_fv;
            fh[k] = joint_output[k][J_DISTANCE] + joint_output[k][J_SPEED] + joint_output[k][J_ALPHA] + joint_output[k][J_BETA] - joint_output[k][J_TURN];
            //fh[k] = joint_mpc_out[k] - joint_output[k][J_TURN];
        }
        mp[k]->current_joint.Fx = fh[k] * cos(theta[k]) + fv[k] * sin(theta[k]);
        mp[k]->current_joint.Fy = -fh[k] * sin(theta[k]) + fv[k] * cos(theta[k]);
        mp[k]->forward_jacobian();
        //腿部使能
        //统一极性，以车辆前进方向为x轴，沿x轴向下角度增加旋转方向为力矩输出正方向
        if (state->flags.leg_enable)
        {
            if (k == RIGHT_JOINT)
            {
                mp[k]->torque_output.wheel = wheel_output[k][W_DISTANCE] + wheel_output[k][W_SPEED] + wheel_output[k][W_ALPHA] + wheel_output[k][W_BETA] + wheel_output[k][W_FF] + wheel_output[k][W_TURN];
                //mp[k]->torque_output.wheel = wheel_mpc_out[k] /* + wheel_output[k][W_FF]*/ + wheel_output[k][W_TURN];
                mp[k]->torque_output.f_joint = mp[k]->current_joint.f_torque;
                mp[k]->torque_output.b_joint = mp[k]->current_joint.b_torque;
            }
            else
            {
                mp[k]->torque_output.wheel = -(wheel_output[k][W_DISTANCE] + wheel_output[k][W_SPEED] + wheel_output[k][W_ALPHA] + wheel_output[k][W_BETA] + wheel_output[k][W_FF] - wheel_output[k][W_TURN]);
                //mp[k]->torque_output.wheel = -(wheel_mpc_out[k]/* + wheel_output[k][W_FF]*/ - wheel_output[k][W_TURN]);
                mp[k]->torque_output.f_joint = -mp[k]->current_joint.f_torque;
                mp[k]->torque_output.b_joint = -mp[k]->current_joint.b_torque;
            }
        }
        else
        {
            mp[k]->torque_output.wheel = 0;
            mp[k]->torque_output.f_joint = 0;
            mp[k]->torque_output.b_joint = 0;
        } 
    }
}

/*******************************************LQR控制***********************************************/
/**
 * @brief  lqr总控
 * @note
 * @param
 * @return  力矩输出
 * @retval  None
 */
void Manipulator_Controller_Classdef::lqr_adjust(Manipulator_Classdef* _mp, lqrCalculater<6, 2>* _lqr_cal)
{
    /*数组打包*/
    lqr_state_config(_mp, lqr_target, lqr_current);
    /*变量写入控制器*/
    _lqr_cal->updateData(lqr_target, lqr_current, upper::constrain(_mp->current_joint.pendulum.length, 0.13f, 0.4f));
    /*控制器结算*/
    _lqr_cal->adjust();
}
/**
 * @brief  lqr状态空间变量设置
 * @note    把状态空间变量打包到给定数组当中
 * @param   目标状态1X6向量，当前状态1X6向量，状态使能1X6向量
 * @return
 * @retval  None
 */
void Manipulator_Controller_Classdef::lqr_state_config(Manipulator_Classdef* mp, float* state_target, float* state_current)
{
    state_target[0] = state->target_location.y;
    state_current[0] = state->current_location.y;
    state_target[1] = state->target_speed.y;
    state_current[1] = state->current_speed.y;
    state_target[2] = mp->target_joint.pendulum.angle;
    state_current[2] = mp->current_joint.pendulum.angle;
    state_target[3] = 0;
    state_current[3] = mp->current_joint.pendulum.dangle;
    state_target[4] = state->target_pos.pitch;
    state_current[4] = state->current_pos.pitch;
    state_target[5] = 0;
    state_current[5] = state->current_av.pitch;

    for (int n = 0; n < 6; n++)
    {
        if (wheel_enable_list[n] == false) {
            state_current[n] = state_target[n] = 0;
        }
        /*if (joint_enable_list[n] == false) {
            joint_output[k][n] = 0;
        }*/
    }
}
// mpc总控
void Manipulator_Controller_Classdef::mpc_adjust(Manipulator_Classdef* _mp, MPC_CALC* _mpcCal, modelFit<6, 6, 3>* _A, modelFit<6, 3, 3>* _B)
{
    mpc_state_config(_mp, mpc_target, mpc_current);
    MATRIX A = model_A->modelGenerate(_mp->current_joint.pendulum.length);
    MATRIX B = model_B->modelGenerate(_mp->current_joint.pendulum.length);
    MATRIX Q(6);
    MATRIX R(3);
    MATRIX F(6);
    MATRIX lb(1, 3);
    MATRIX ub(1, 3);
    MATRIX X(6, 1);
    MATRIX Y(6, 1);
    Q.setArray(user->Q, sizeof(user->Q) / sizeof(real_t));
    R.setArray(user->R, sizeof(user->R) / sizeof(real_t));
    lb.setArray(user->lb, sizeof(user->lb) / sizeof(real_t));
    ub.setArray(user->ub, sizeof(user->ub) / sizeof(real_t));
    X.setArray(mpc_current, sizeof(mpc_current) / sizeof(real_t));
    Y.setArray(mpc_target, sizeof(mpc_target) / sizeof(real_t));
    F = Q * 2;
    X.print();
    _mpcCal->setConstrain(lb, ub);
    _mpcCal->mpc_update(Y, X, 20, 0.008);
    _mpcCal->mpc_init(A, B, Q, R, F);
    _mpcCal->mpc_solve();
    _mpcCal->compare_storage();
}

//mpc+kalman联合控制
void Manipulator_Controller_Classdef::mpc_adjust_kalman(Manipulator_Classdef* _mp, MPC_CALC* _mpcCal, modelFit<6, 6, 3>* _A, modelFit<6, 3, 3>* _B, KALMAN_CALC* _kalmanF)
{
    mpc_state_config(_mp, mpc_target, mpc_current);
    MATRIX A = model_A->modelGenerate(_mp->current_joint.pendulum.length);
    MATRIX B = model_B->modelGenerate(_mp->current_joint.pendulum.length);
    MATRIX Q(6);
    MATRIX R(3);
    MATRIX F(6);
    MATRIX lb(1, 3);
    MATRIX ub(1, 3);
    MATRIX X(6, 1);
    MATRIX Y(6, 1);
    Q.setArray(user->Q, sizeof(user->Q) / sizeof(real_t));
    R.setArray(user->R, sizeof(user->R) / sizeof(real_t));
    lb.setArray(user->lb, sizeof(user->lb) / sizeof(real_t));
    ub.setArray(user->ub, sizeof(user->ub) / sizeof(real_t));
    X.setArray(mpc_current, sizeof(mpc_current) / sizeof(real_t));
    Y.setArray(mpc_target, sizeof(mpc_target) / sizeof(real_t));
    F = Q * 2;
    _kalmanF->setFunc(A.getArray(), B.getArray(), user->kalman_H);
    _kalmanF->setConv(user->kalman_Q, user->kalman_R);
    _kalmanF->f(out.getArray(), X.getArray());
    _mpcCal->setConstrain(lb, ub);
    /*for (int i = 0; i < 6; i++)
    {
        X.setElement(i, 0, _kalmanF->getOut().getElement(i, 0));
    }*/
    //_kalmanF->getOut().print();
    _mpcCal->mpc_update(Y, X, 20, 0.008);
    _mpcCal->mpc_init(A, B, Q, R, F);
    _mpcCal->mpc_solve();
    _mpcCal->compare_storage();
}

//mpc接入外部控制器输出
void Manipulator_Controller_Classdef::mpc_adjust(Manipulator_Classdef* _mp, MPC_CALC* _mpcCal, modelFit<6, 6, 3>* _A, modelFit<6, 3, 3>* _B, real_t ext_uk[3])
{
    mpc_state_config(_mp, mpc_target, mpc_current);
    MATRIX A = model_A->modelGenerate(_mp->current_joint.pendulum.length);
    MATRIX B = model_B->modelGenerate(_mp->current_joint.pendulum.length);
    MATRIX Q(6);
    MATRIX R(3);
    MATRIX F(6);
    MATRIX X(6, 1);
    MATRIX Y(6, 1);
    Q.setArray(user->Q, sizeof(user->Q) / sizeof(real_t));
    R.setArray(user->R, sizeof(user->R) / sizeof(real_t));
    X.setArray(mpc_current, sizeof(mpc_current) / sizeof(real_t));
    Y.setArray(mpc_target, sizeof(mpc_target) / sizeof(real_t));
    F = Q;
    _mpcCal->mpc_update(Y, X);
    _mpcCal->mpc_init(A, B, Q, R, F);
    _mpcCal->mpc_solve(ext_uk);
}

// mpc状态空间变量设置
void Manipulator_Controller_Classdef::mpc_state_config(Manipulator_Classdef* mp, real_t* state_target, real_t* state_current)
{
    state_target[0] = state->target_location.y;
    state_current[0] = state->current_location.y;
    if (mp->target_joint.pendulum.length > 0.25)
    {
        state_target[1] = 0.5 * state->target_speed.y;
    }
    else
    {
        state_target[1] = state->target_speed.y;
    }
    state_current[1] = state->current_speed.y;
    state_target[2] = mp->target_joint.pendulum.angle;
    state_current[2] = mp->current_joint.pendulum.angle;
    state_target[3] = 0;
    state_current[3] = mp->current_joint.pendulum.dangle;
    state_target[4] = state->target_pos.pitch;
    state_current[4] = state->current_pos.pitch;
    state_target[5] = 0;
    state_current[5] = state->current_av.pitch;

    for (int n = 0; n < 6; n++)
    {
        if (wheel_enable_list[n] == false) {
            state_current[n] = state_target[n] = 0;
        }
        /*if (joint_enable_list[n] == false) {
            joint_output[k][n] = 0;
        }*/
    }
}

/*******************************************轮控制***********************************************/

/**
 * @brief  lqr转向环
 * @note
 * @param
 * @return  力矩输出
 * @retval  None
 */
float Manipulator_Controller_Classdef::wheel_turn_adjust()
{
    static float last_target = 0;
    if (state->target_av.yaw >= last_target)
    {
        if (abs(state->target_av.yaw - last_target) > user->turn_step)
        {
            last_target += user->turn_step;
        }
        else
        {
            last_target = state->target_av.yaw;
        }
    }
    else if (state->target_av.yaw < last_target)
    {
        if (abs(state->target_av.yaw - last_target) > user->turn_step)
        {
            last_target -= user->turn_step;
        }
        else
        {
            last_target = state->target_av.yaw;
        }
    }
    else
    {
    }
    w_turn_pid->target = last_target;
    w_turn_pid->current = state->current_av.yaw;
    w_turn_pid->Adjust(0);
    return w_turn_pid->out;
}

/**
 * @brief  lqr前馈环
 * @note
 * @param
 * @return  力矩输出
 * @retval  None
 */
float Manipulator_Controller_Classdef::wheel_feedforward_adjust(Manipulator_Classdef* _mp)
{
    /*float error = 0 - _mp->overall.angle;*/
    float error = 0 - state->current_pos.pitch;
    float debug_feedforward = -_mp->overall.mass * 9.8f * _mp->overall.length;
    /*float debug_feedforward = -0.5*_mp.body.mass * 9.8 * (_mp.current_joint.pendulum.length+_mp.body.length);*/
    /*float debug_feedforward = -0.5 * _mp.body.mass * 9.8 * _mp->overall.length;*/
    /*float debug_feedforward = -_mp->overall.mass * 9.8 * (_mp.current_joint.pendulum.length + _mp.body.length);*/
    if (error > 0)
    {
        return debug_feedforward * sin(abs(error));
    }
    else
    {
        return -debug_feedforward * sin(abs(error));
    }
}

/*********************************************腿控制****************************************************/

/**
 * @brief  车体转向内倾
 * @note    用角速度，速度，腿长三个变量拟合增益
 * @param
 * @return  roll轴角度目标值
 * @retval  None
 */
float Manipulator_Controller_Classdef::turn_adaption_adjust()
{
    if (state->flags.sport_flag && abs(state->current_av.yaw) > 0.2f)
    {
        float out;
        if (state->current_speed.y >= 0)
        {
            out = -user->turn_adaption_kp * (state->current_av.yaw / abs(state->current_av.yaw)) * (0.4f + abs(state->current_av.yaw) * 1.1f / 1.5f) * pow((mp[RIGHT_JOINT]->current_joint.pendulum.length + mp[LEFT_JOINT]->current_joint.pendulum.length), 2) * (0.7f + abs(state->current_speed.y) * 0.6f / 3.f);
        }
        else if (state->current_speed.y < 0)
        {
            out = user->turn_adaption_kp * (state->current_av.yaw / abs(state->current_av.yaw)) * (0.4f + abs(state->current_av.yaw) * 1.1f / 1.5f) * pow((mp[RIGHT_JOINT]->current_joint.pendulum.length + mp[LEFT_JOINT]->current_joint.pendulum.length), 2) * (0.7f + abs(state->current_speed.y) * 0.6f / 3.f);
        }
        return out;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief  车体转向内倾2
 * @note    用角速度，速度，腿长三个变量拟合增益
 * @param
 * @return  两轮杆方向力矩差分
 * @retval  None
 */
float Manipulator_Controller_Classdef::turn_adaption_adjust2()
{
    if (state->flags.sport_flag && abs(state->current_av.yaw) > 0.2f)
    {
        float out;
        if (state->current_speed.y >= 0)
        {
            out = -user->turn_adaption_kp2 * (state->current_av.yaw / abs(state->current_av.yaw)) * (0.4f + abs(state->current_av.yaw) * 1.1f / 1.5f) * pow((mp[RIGHT_JOINT]->current_joint.pendulum.length + mp[LEFT_JOINT]->current_joint.pendulum.length), 2) * (0.7f + abs(state->current_speed.y) * 0.8f / 3.f);
        }
        else if (state->current_speed.y < 0)
        {
            out = user->turn_adaption_kp2 * (state->current_av.yaw / abs(state->current_av.yaw)) * (0.4f + abs(state->current_av.yaw) * 1.1f / 1.5f) * pow((mp[RIGHT_JOINT]->current_joint.pendulum.length + mp[LEFT_JOINT]->current_joint.pendulum.length), 2) * (0.7f + abs(state->current_speed.y) * 0.8f / 3.f);
        }
        return out;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief  车体转向内倾3
 * @note    用roll倾角拟合增益
 * @param
 * @return  两轮杆方向力矩差分
 * @retval  None
 */
float Manipulator_Controller_Classdef::turn_adaption_adjust3()
{
    if (state->flags.sport_flag && abs(state->current_av.yaw) > 0.2f)
    {
        // return -(current_pos.roll- target_pos.roll) * turn_adaption_kp3*(0.4 + abs(current_av.yaw) * 1.1 / 1.5) * pow((current_right_joint.pendulum.length + current_left_joint.pendulum.length), 2) * (0.7 + abs(current_speed.y) * 0.8 / 3.);
        return -(state->current_pos.roll - state->target_pos.roll) * user->turn_adaption_kp3;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief  车体roll轴平衡
 * @note
 * @param
 * @return  高度双腿差分值
 * @retval  None
 */
float Manipulator_Controller_Classdef::roll_keep_adjust()
{
    j_roll_keep_pid->target = state->current_pos.roll;
    j_roll_keep_pid->current = state->target_pos.roll;
    j_roll_keep_pid->Adjust(0, state->current_av.roll);
    return j_roll_keep_pid->out;
}

/**
 * @brief  轮腿高度自适应
 * @note
 * @param
 * @return  得出某个轮腿在y方向上的力
 * @retval  None
 */
float Manipulator_Controller_Classdef::length_keep_adjust(Manipulator_Classdef* _mp)
{
    float length_target = upper::constrain(_mp->target_joint.pendulum.length, 0.13, 0.4);
    float length_kd;
    float length_kp;
    float length_keep_feedforward = -0.5 * mp[RIGHT_JOINT]->body.mass * 9.8;
    //length_keep_feedforward = 0;
    if (state->flags.leg_length_step) //轮腿高度的非阶跃响应
    {
        if (length_target > _mp->target_joint.pendulum.last_length)
        {
            _mp->target_joint.pendulum.last_length += user->length_upstep;
        }
        else if (length_target < _mp->target_joint.pendulum.last_length)
        {
            _mp->target_joint.pendulum.last_length -= user->length_downstep;
        }
        else
        {
        }
        length_kd = user->length_keep_kd;
        length_kp = user->length_keep_kp;
    }
    else //阶跃响应
    {
        _mp->target_joint.pendulum.last_length = length_target;
        if (state->flags.kd_inhibition)
        {
            length_kd = user->length_keep_kd / 2.f;
            length_kp = user->length_keep_kp * 7.f / 6.f;
        }
        else
        {
            length_kd = user->length_keep_kd;
            length_kp = user->length_keep_kp;
        }
    }
    j_length_keep_pid->kp = length_kp;
    j_length_keep_pid->kd = length_kd;
    j_length_keep_pid->target = -_mp->target_joint.pendulum.last_length;
    j_length_keep_pid->current = -_mp->current_joint.pendulum.length;
    j_length_keep_pid->Adjust(0, _mp->current_joint.pendulum.dlength);
    return length_keep_feedforward + j_length_keep_pid->out;
}

/**
 * @brief  关节转向环
 * @note    用于避免两腿开叉
 * @param
 * @return  末端力
 * @retval  None
 */
float Manipulator_Controller_Classdef::joint_turn_adjust(Manipulator_Classdef* _mp_right, Manipulator_Classdef* _mp_left)
{
    j_turn_pid->target = state->target_av.yaw;
    j_turn_pid->current = state->current_av.yaw;
    j_turn_pid->Adjust(0);
    j_follow_pid->target = _mp_right->current_joint.pendulum.angle;
    j_follow_pid->current = _mp_left->current_joint.pendulum.angle;
    j_follow_pid->Adjust(0);
    return j_follow_pid->out - j_turn_pid->out;
}

/*******************************功能动作函数**************************************/
/**
 * @brief  车体弹跳函数
 * @note
 * @param
 * @return  末端力
 * @retval  None
 */
void Manipulator_Controller_Classdef::jump_adjust()
{
    static int count = 0;
    static bool is_build = false;
    /*cout << right_mp->current_joint.pendulum.length << endl;
    cout << left_mp->current_joint.pendulum.length << endl;*/
    if (state->flags.jump_flag)
    {
        count += 120;
        if (mp[RIGHT_JOINT]->current_joint.pendulum.length > 0.22f || mp[LEFT_JOINT]->current_joint.pendulum.length > 0.22f)
        {
            is_build = true;
        }
        else
        {
            is_build = false;
        }
        state->flags.jump_flag = false;
    }
    else
    {
        count--;
    }
    count = upper::constrain(count, 0, 120);
    if (is_build)
    {
        if (count % 120 > 90)
        {
            mp[RIGHT_JOINT]->target_joint.pendulum.length = 0.2f;
            mp[LEFT_JOINT]->target_joint.pendulum.length = 0.2f;
            state->flags.kd_inhibition = false;
        }
        else if (count % 120 < 90 && count % 120 > 70)
        {
            mp[RIGHT_JOINT]->target_joint.pendulum.length = 0.2f + 0.3f;
            mp[LEFT_JOINT]->target_joint.pendulum.length = 0.2f + 0.3f;
            state->flags.kd_inhibition = true;
        }
        else if (count % 120 < 70 && count % 120 > 40)
        {
            mp[RIGHT_JOINT]->target_joint.pendulum.length = 0.2f - 0.1f;
            mp[LEFT_JOINT]->target_joint.pendulum.length = 0.2f - 0.1f;
            state->flags.kd_inhibition = true;
        }
        else
        {
            state->flags.kd_inhibition = false;
        }
        if (count > 40)
        {
            state->flags.leg_length_step = false;
        }
        else
        {
            state->flags.leg_length_step = true;
            is_build = false;
        }
    }
    else
    {
        if (count % 120 > 100)
        {
            mp[RIGHT_JOINT]->target_joint.pendulum.length += 0.3f;
            mp[LEFT_JOINT]->target_joint.pendulum.length += 0.3f;
            state->flags.kd_inhibition = true;
        }
        else if (count % 120 < 100 && count % 120 > 70)
        {
            mp[RIGHT_JOINT]->target_joint.pendulum.length -= 0.1f;
            mp[LEFT_JOINT]->target_joint.pendulum.length -= 0.1f;
            state->flags.kd_inhibition = true;
        }
        else
        {
            state->flags.kd_inhibition = false;
        }
        if (count > 70)
        {
            state->flags.leg_length_step = false;
        }
        else
        {
            state->flags.leg_length_step = true;
            is_build = false;
        }
    }
}

#endif
