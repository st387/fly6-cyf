#include "zf_common_headfile.h"


float tof_real=0;
#define MY_PI 3.14159265358979323846
#define alpha 0.2
/***************************************************************

 *************************************************************/
int dl1b_distance_use;

float tof_speed = 0.0f;
int tof_change=1;
void tof_filtering(void)
{
  if(dl1b_distance_mm>8000 )
  {
    tof_change=1;
    return;
  }
  dl1b_distance_use = alpha * dl1b_distance_mm + (1-alpha) *dl1b_distance_use;
  tof_change=0;
}

void tof_correct(float my_roll, float my_pitch, float L)
{
    static float last_tof = 0.0f;
    
    float phi   = fabsf(my_roll)  * (float)MY_PI / 180.0f;
    float theta = fabsf(my_pitch)* (float)MY_PI / 180.0f;
    tof_real=L * cosf(theta) * cosf(phi);
    if(!tof_change)
    {
      tof_speed = (tof_real - last_tof);
      last_tof = tof_real;
    }   
}
