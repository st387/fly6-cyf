#include "zf_common_headfile.h"

#define alpha 0.1

int Xerr_init=0;
int Yerr_init=0;
 float pmw3901_delta_x_use;
 float pmw3901_delta_y_use;
void  pmw_err_init(void)
{
    int Xdata_all=0;
    int Ydata_all=0;
    for(int i=0;i<100;i++)
    {
        pmw3901_get_motion();
       Xdata_all += pmw3901_delta_x ;
       Ydata_all += pmw3901_delta_y ;
        system_delay_ms(20);
    }
    Xerr_init=Xdata_all;
    Yerr_init=Ydata_all;
}

void pmw_dataget(void)
{
  pmw3901_get_motion();
  static float pmw3901_delta_x_last=0;
  static float pmw3901_delta_y_last=0;
  pmw3901_delta_x -= Xerr_init;
  pmw3901_delta_y -= Yerr_init;
  pmw3901_delta_x_use = alpha * pmw3901_delta_x + (1-alpha) *pmw3901_delta_x_last;
  pmw3901_delta_y_use = alpha * pmw3901_delta_y + (1-alpha) *pmw3901_delta_y_last;
//  pmw3901_delta_x_last=pmw3901_delta_x_use;
//  pmw3901_delta_y_last=pmw3901_delta_y_use;
}

