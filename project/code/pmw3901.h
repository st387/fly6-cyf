#ifndef _pmw3901_h_
#define _pmw3901_h_

#include "zf_common_typedef.h"

extern int Xerr_init;
extern int Yerr_init;
extern float pmw3901_delta_x_use;
extern float pmw3901_delta_y_use;

void pmw_err_init(void);
void pmw_dataget(void);


#endif