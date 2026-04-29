#include "zf_common_headfile.h"


PID_F ANGLE_Pitch_PID;//角度
PID_F ANGLE_Roll_PID;
PID_F ANGLE_Yaw_PID;
float Yaw_target;

PID_F ASPEEDF_Pitch_PID;//角速度
PID_F ASPEEDF_Roll_PID;
PID_F ASPEEDF_Yaw_PID;
PID_I ASPEED_Pitch_PID;//角速度
PID_I ASPEED_Roll_PID;
PID_I ASPEED_Yaw_PID;

PID_F HIGH_speed;
PID_F HIGH_loca;
PID_I HIGH_PID;//高度
int High_target=100;

PID_F Locat_x_PID; 
PID_F Locat_y_PID; 
float imageYerr=0;

//增量式
void PID_I_Init(PID_I *pid, float p, float i, float d, float maxI, float maxOut)
{
    pid->kp = p;
    pid->ki = i;
    pid->kd = d;
    pid->maxIout=maxI;
    pid->maxOutput = maxOut;
}
void PID_I_Calculate(PID_I *pid, float present, float target)
{
    float e  = target - present;
    float de = e - pid->lastError;
    float d2e = e - 2.0f*pid->lastError + pid->last2Error;

    float integral=pid->ki*e;
    if(integral > pid->maxIout)integral = pid->maxIout;
    else if(integral < -pid->maxIout) integral = -pid->maxIout;
    
    float delta = pid->kp*de
                + integral
                + pid->kd*d2e;

    pid->output += delta;

    if(pid->output >  pid->maxOutput) pid->output =  pid->maxOutput;
    if(pid->output < -pid->maxOutput) pid->output = -pid->maxOutput;

    pid->last2Error = pid->lastError;
    pid->lastError  = e;
}

//位置式
void PID_F_Init(PID_F *pid, float p, float i, float d, float maxI, float maxOut)
{
    pid->kp = p;
    pid->ki = i;
    pid->kd = d;
    pid->maxIntegral = maxI;
    pid->maxOutput = maxOut;
}

void PID_F_Calculate(PID_F *pid, float present, float target)
{
    float pout, dout;

    pid->error = target - present;

    pout = pid->kp * pid->error;
    pid->integral += pid->ki * pid->error;
    if(pid->integral > pid->maxIntegral) pid->integral = pid->maxIntegral;
    else if(pid->integral < -pid->maxIntegral) pid->integral = -pid->maxIntegral;

    dout = pid->kd * (pid->error - pid->lastPresent);
    pid->lastPresent = pid->error ;
    pid->output = pout + pid->integral + dout;

    if(pid->output > pid->maxOutput) pid->output = pid->maxOutput;
    else if(pid->output < -pid->maxOutput) pid->output = -pid->maxOutput;

}