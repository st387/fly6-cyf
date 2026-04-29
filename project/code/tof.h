#ifndef _tof_h_
#define _tof_h_

#include "zf_common_typedef.h"

extern float tof_real;
extern int dl1b_distance_use;
extern float tof_speed;
extern int tof_change;

void tof_correct(float my_roll, float my_pitch, float L);
void tof_filtering(void);


#endif