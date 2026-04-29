#include "zf_common_headfile.h"

mon_control mon_out;

void motor_init(void)
{
   pwm_init(TCPWM_CH28_P10_0 , 50, 0);  
   pwm_init(TCPWM_CH29_P10_1 , 50, 0);  
   pwm_init(TCPWM_CH48_P14_0 , 50, 0);  
   pwm_init(TCPWM_CH49_P14_1 , 50, 0);  
}

uint32_t motor_limit(float in,float max,float min)
{
  if(in>max)       return (uint32_t)max;
  else if(in<min)  return (uint32_t)min;
  else             return (uint32_t)in;
}



