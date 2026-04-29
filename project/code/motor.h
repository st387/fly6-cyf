#ifndef _motor_h_
#define _motor_h_

#include "zf_common_typedef.h"

#define mon_Lup      TCPWM_CH49_P14_1
#define mon_Ldown    TCPWM_CH48_P14_0
#define mon_Rup      TCPWM_CH29_P10_1
#define mon_Rdown    TCPWM_CH28_P10_0

typedef struct
{
    float leftup;
    float leftdown;
    float rightup;
    float rightdown;
}mon_control;

extern mon_control mon_out;

void motor_init(void);
uint32_t motor_limit(float in,float max,float min);

#endif