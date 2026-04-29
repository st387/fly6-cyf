#ifndef _imu_h_
#define _imu_h_

#include "zf_common_typedef.h"

extern float angle;
typedef struct
{
    float x;
    float y;
    float z;
    float x_PI;//½ÇËÙ¶È»¡¶ÈÖÆ
    float y_PI;
    float z_PI;
    float x_filtered;//ÂË²¨
    float y_filtered;
    float z_filtered;

    float err[3];
}XGYRO,*XGYR;

typedef struct
{
    float x;
    float y;
    float z;
    float ax_filtered;//¼ÓËÙ¶È¼ÆÂË²¨
    float ay_filtered;
    float az_filtered;
}XACC,*XAC;

typedef struct
{
        float Pitch;//¸©Ñö½Ç
        float Roll;//ºá¹ö½Ç
        float Yaw;//Æ«º½½Ç

        float x_true;
        float y_true;
        float z_true;
        XGYRO  Gyro;
        XACC  Acc;
}ANGLE,*ANG;

extern ANGLE my_angle;
extern ANGLE angle_ekf;

uint8 Err_Reduce(void);
void imu660ra_value_get(void);
void KF_6_Axis(float acc_x,float acc_y,float acc_z,
                float gyro_x,float gyro_y,float gyro_z);
void IMU_Update(float gx, float gy, float gz,
                    float ax, float ay, float az);


//void ekf_6dof_init(void);
uint8_t ekf_6dof_update(float acc_x, float acc_y, float acc_z,
                        float gyro_x, float gyro_y, float gyro_z);

#endif
