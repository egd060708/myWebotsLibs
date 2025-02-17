/*! @file state_data.h
 *  @brief Data structs for motion commands storage
 *
 *  This file contains the State_Data_Classdef and state_timer class,
 *  which provide ways to store motion commands.
 *  此文件用于储存车辆整体控制参量及标志位
 */
#ifndef _STATE_DATA_H_
#define _STATE_DATA_H_

#include "Upper_Public.h"


typedef uint32_t(*getSystemTick)(void);//函数指针，用于接入不同单片机获取时间栈的函数

class state_timer
{
public:
    static uint8_t getMicroTick_regist(uint32_t(*getTick_fun)(void));//获取当前时间函数接口
    static getSystemTick Get_SystemTick;   //获取时间的函数
    float dt = 0;				                //时间微分
    uint32_t last_time; 	                //记录上次时间
    uint8_t UpdataTimeStamp(void);          //时间栈更新
};

/*线性参数*/
struct Linear_Structdef
{
    float x;
    float y;
    float z;
};
/*角度参数(统一用弧度制)*/
struct Angular_Structdef
{
    float pitch;
    float yaw;
    float roll;
};
/*标志位*/
struct StateFlag_Structdef
{
    bool sport_flag = false;//移动标志位
    bool break_flag = false;
    bool leap_flag = false;
    bool slope_flag = false;
    bool pull_up_flag = false;//采用滞回比较器
    bool distance_flag = false;//路程环的使能位
    bool leg_enable = false;//遥控保护
    bool r_single_roll = false;//是否是单边roll轴控制
    bool l_single_roll = false;
    bool weightlessness = false;
    bool is_weightless_check = true;//腾空检测开关
    bool tiny_weightlessness = false;
    bool leg_length_step = true;//判断是否需要进行腿长变化率目标值限制
    bool jump_flag = false;//弹跳开关
    bool kd_inhibition = false;//抑制支撑环kd
};

/*状态类*/
class State_Data_Classdef :public state_timer
{
public:
    State_Data_Classdef();
    State_Data_Classdef(float _dt) { dt = _dt; }

    /*状态更新函数*/
    void timeStamp_update(float _dt = 0);
    void target_update(float target_speed, float target_turn);//目标值更新
    void current_location_update(float x, float y, float z);//位置更新
    void current_speed_update(float x, float y, float z);//速度更新
    void current_acc_update(float x, float y, float z);//加速度更新
    void current_pos_update(float pitch, float yaw, float roll);//角度更新
    void current_av_update(float pitch, float yaw, float roll);//角速度更新
    /*状态判断函数*/
    void sport_adaption();//用于运动状态检测
    void weightlessness_check();//用于检测跳台阶，飞坡等需要缓冲的腾空检测
    void tiny_weightlessness_check();//用于上下坡，伸长缩短腿时保持平衡的腾空检测

    Linear_Structdef target_location = { 0 };
    Linear_Structdef current_location = { 0 };

    Linear_Structdef target_speed = { 0 };
    Linear_Structdef current_speed = { 0 };

    Linear_Structdef target_acc = { 0 };
    Linear_Structdef current_acc = { 0 };

    Angular_Structdef target_pos = { 0 };
    Angular_Structdef current_pos = { 0 };

    Angular_Structdef target_av = { 0 };
    Angular_Structdef current_av = { 0 };

    StateFlag_Structdef flags = { 0 };//标志位手动一个个配置
};


/******************************* timer *******************************/
getSystemTick state_timer::Get_SystemTick = NULL;//静态变量必须实现

uint8_t state_timer::UpdataTimeStamp(void)
{
    uint32_t now_time;

    /*Check `Get_SystemTick` */
    if (state_timer::Get_SystemTick != NULL)
    {
        /*Convert to system time*/
        if (last_time == 0)
        {
            last_time = state_timer::Get_SystemTick();
            return 1;//第一步先不跑
        }
        now_time = state_timer::Get_SystemTick();

        /*Overflow*/
        if (now_time < last_time)
            dt = (float)(now_time + (0xFFFFFFFF - last_time));
        else
            dt = (float)(now_time - last_time);

        last_time = now_time;

        dt *= (float)0.000001;

        return 0;
    }
    else {
        dt = 0.001f;//如果没有则默认1ms避免出现错误
        return 1;
    }
}

/**
 * @brief  Regist get time function(1Tick = 1us)
 * @param  realTime_fun: Pointer of function to get system real time
 * @retval 1: success
           0: error input param
 * @author
 */
uint8_t state_timer::getMicroTick_regist(uint32_t(*getTick_fun)(void))
{
    if (getTick_fun != NULL)
    {
        state_timer::Get_SystemTick = getTick_fun;
        return 1;
    }
    else
        return 0;
}



/*********************************** state ***************************************/
State_Data_Classdef::State_Data_Classdef()
{

}

/*时间栈更新*/
//如果输入参数则可以自定义控制周期
void State_Data_Classdef::timeStamp_update(float _dt)
{
    if (_dt == 0)
    {
        this->UpdataTimeStamp();
    }
    else
    {
        this->dt = _dt;
    }
}

/*数据更新*/
void State_Data_Classdef::target_update(float target_speed_, float target_turn_)
{
    target_speed.y = target_speed_;
    target_av.yaw = target_turn_;
}

void State_Data_Classdef::current_location_update(float x, float y, float z)
{
    current_location.x = x;
    current_location.y = y;
    current_location.z = z;
}

void State_Data_Classdef::current_speed_update(float x, float y, float z)
{
    current_speed.x = x;
    current_speed.y = y;
    current_speed.z = z;
}

void State_Data_Classdef::current_acc_update(float x, float y, float z)
{
    current_acc.x = x;
    current_acc.y = y;
    current_acc.z = z;
}

void State_Data_Classdef::current_av_update(float pitch, float yaw, float roll)
{
    current_av.pitch = pitch;
    current_av.yaw = yaw;
    current_av.roll = roll;
}

void State_Data_Classdef::current_pos_update(float pitch, float yaw, float roll)
{
    current_pos.pitch = pitch;
    current_pos.yaw = yaw;
    current_pos.roll = roll;
}

/*状态更新函数*/
void State_Data_Classdef::sport_adaption()
{
    //先进行运动状态判断
    if (abs(current_speed.y) > 1.0)
    {
        flags.sport_flag = true;

        if (abs(target_speed.y) < 0.1 * abs(current_speed.y))
        {
            flags.break_flag = true;
        }
        else
        {
            flags.break_flag = false;
        }
    }
    else
    {
        flags.sport_flag = flags.break_flag = false;

        if (abs(current_speed.y) < 0.6 && target_speed.y == 0)
        {
            flags.distance_flag = true;
        }
        else
        {
            flags.distance_flag = false;
        }
    }

    /*路程环通断检测*/
    static int distance_delay = 10;
    static int distance_count = 0;        //目标距离设置计数
    static bool distance_enable = false;
    if (flags.distance_flag)
    {//每一次触发路程环都只设置一次目标值，延迟可以调
        if (!distance_enable)	//如果距离环被使能了，就不再更新距离目标值
        {
            distance_count++;
            if (distance_count == distance_delay)
            {
                distance_enable = true;
            }
            else
            {
            }
            target_location.y = current_location.y;
        }
        else
        {

        }
    }
    else
    {
        target_location.y = current_location.y;
        distance_count = 0;
        distance_enable = false;
    }
}


/**
 * @brief  腾空检测
 * @note
 * @param
 * @return
 * @retval  None
 */
void State_Data_Classdef::weightlessness_check()
{
    static int8_t count = 0;//用于计数腾空时间
    if (current_acc.y < 5 && abs(current_pos.pitch) < 0.2)
    {
        count += 3;
    }
    else
    {
        count -= 8;
    }
    count = upper::constrain(count, 0, 80);
    if (count > 30)
    {
        flags.weightlessness = true;
        flags.leg_length_step = false;
    }
    else
    {
        flags.weightlessness = false;
        flags.leg_length_step = true;
    }
}

/**
 * @brief  失重检测(小幅腾空)
 * @note
 * @param
 * @return
 * @retval  None
 */
void State_Data_Classdef::tiny_weightlessness_check()
{
    static int8_t count = 0;//用于计数腾空时间
    if (current_acc.y < 5)
    {
        count += 2;
    }
    else
    {
        count -= 2;
    }
    if (flags.weightlessness || flags.break_flag)
    {
        count = 0;
    }
    count = upper::constrain(count, 0, 20);
    if (count > 0)
    {
        flags.tiny_weightlessness = true;
    }
    else
    {
        flags.tiny_weightlessness = false;
    }
}

#endif
