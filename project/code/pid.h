#ifndef _pid_h_
#define _pid_h_

#include "zf_common_typedef.h"

typedef struct
{
    float kp, ki, kd;
    float maxIout;
    float error, lastError, last2Error;
    float output, maxOutput;
}PID_I;

typedef struct
{
    float kp, ki, kd;
    float error, lastError,lastPresent;
    float integral, maxIntegral;
    float output, maxOutput;
}PID_F;

extern PID_F ANGLE_Pitch_PID;//角度
extern PID_F ANGLE_Roll_PID;
extern PID_F ANGLE_Yaw_PID;
extern float Yaw_target;

extern PID_F ASPEEDF_Pitch_PID;//角速度
extern PID_F ASPEEDF_Roll_PID;
extern PID_F ASPEEDF_Yaw_PID;
extern PID_I ASPEED_Pitch_PID;//角速度
extern PID_I ASPEED_Roll_PID;
extern PID_I ASPEED_Yaw_PID;

extern PID_F HIGH_speed;
extern PID_F HIGH_loca;

extern PID_I HIGH_PID;//高度
extern int High_target;

extern PID_F Locat_x_PID;  
extern PID_F Locat_y_PID;  
extern float imageYerr;

void PID_I_Init(PID_I *pid, float p, float i, float d, float maxI, float maxOut);
void PID_I_Calculate(PID_I *pid, float present, float target);
void PID_F_Init(PID_F *pid, float p, float i, float d, float maxI, float maxOut);
void PID_F_Calculate(PID_F *pid, float present, float target);


#endif