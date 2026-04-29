/*********************************************************************************************************************
* CYT4BB Opensourec Library 即（ CYT4BB 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 CYT4BB 开源库的一部分
*
* CYT4BB 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          cm7_0_isr
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT4BB
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-1-9      pudding            first version
* 2024-5-14     pudding            新增12个pit周期中断 增加部分注释说明
* 2025-2-4      pudding            优化串口中断逻辑，防止意外干扰导致的卡死问题，优化串口波特率计算逻辑
* 2025-2-4      pudding            新增两个串口接口
********************************************************************************************************************/

#include "zf_common_headfile.h"

// ===== 新增：引用main_cm7_0中的视觉对准状态 =====
extern int yaw_align_ok;
extern int yaw_reset_done;
extern float imagePitchTarget;
extern uint8 image_find_state;

enum
{
    IMAGE_FIND_SEARCH = 0,
    IMAGE_FIND_ALIGN_X = 1,
    IMAGE_FIND_APPROACH = 2,
    IMAGE_FIND_STOP = 3
};

#define base_speed 613  
#define High 950 //高度环目标值

#define high_flag 1    //为0时不加高度环
#define kf_ekf 0       //为0时滤波选择为kf
#define locat_dis   1  //为0不加位置环



#define ASPEED_I_F   0   
//1: 使用位置式控制逻辑 0: 使用增量式控制逻辑
//整个ASPEED_I_F里所有宏定义目前都是服务角速度环
//参数选择为ASPEEDF_Roll_PID位置式，ASPEED_Pitch_PID增量式，
//由于pid函数实现限制，ASPEED_con参数选择修改时，只需要屏蔽并打开对应函数实现即可*******
//增量式和位置式的电机逻辑不一致，因此通过标志位motor_Integration（意为电机融合)来控制，电机融合逻辑//增量和位置分开，没有这个标志位
#if ASPEED_I_F


    #define ASPEED_PID_Calculate PID_F_Calculate
    #define ASPEED_Roll_con   ASPEEDF_Roll_PID
    #define ASPEED_Pitch_con  ASPEEDF_Pitch_PID
                    
    #define Aspeed_flag 1   //位置式来讲，单环和双环的电机融合逻辑一致，因此当使用位置式时不需要此标志位，在此只做占位
                            //也可以用这个标志位，当仅验证角速度环时给0，可以避免修改pid输入  
#else
    #define ASPEED_PID_Calculate PID_I_Calculate
    #define ASPEED_Roll_con   ASPEED_Roll_PID
    #define ASPEED_Pitch_con  ASPEED_Pitch_PID
    #define ASPEED_Yaw_con    ASPEEDF_Yaw_PID
    
    #define Aspeed_flag 1  //为0时单调角速度环

#endif

#define high_I_F 0 //为1时为增量式高度环

#if high_I_F 
#define HIGH_out HIGH_PID.output
#define high_I_F_flag 1
#else 
#define HIGH_out HIGH_speed.output
#define high_I_F_flag 0
#endif

int Yaw_to0=0;
// **************************** PIT中断函数 ****************************
void pit0_ch0_isr()                     // 定时器通道 0 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH0);
  
        // ===== 原逻辑保留：开始 =====
//    Yaw_to0++;
//    if(imageYerr==0 && Yaw_target!=0) 
//    {
//      Yaw_to0=0;
//      my_angle.Yaw=0;
//    }
    // ===== 原逻辑保留：结束 =====

    // ===== 新增：只有未基本对准灯时才允许Yaw清零 =====
    Yaw_to0++;
    if (!yaw_align_ok && Yaw_target != 0)
    {
        Yaw_to0 = 0;
        my_angle.Yaw = 0;
    }
    //---------------------------数据处理-------------------------------------------------------
    dl1b_get_distance();
    tof_filtering();
    imu660ra_value_get();
//    icm42688_value_get();
    if(dl1b_distance_use>1300) dl1b_distance_use=1300;//由于逐飞库问题，当dl1b>1300+时，数值会变到8000+
    Gz = imu660ra_gyro_transition(imu660ra_gyro_z);
    my_angle.Yaw+=Gz*0.003;
    if(kf_ekf) {ekf_6dof_update(my_angle.Acc.ax_filtered,my_angle.Acc.ay_filtered,my_angle.Acc.az_filtered,my_angle.Gyro.x_PI,my_angle.Gyro.y_PI,my_angle.Gyro.z_PI);
                tof_correct(angle_ekf.Roll,angle_ekf.Pitch,dl1b_distance_use);}//ekf
    else       {KF_6_Axis(my_angle.Acc.ax_filtered,my_angle.Acc.ay_filtered,my_angle.Acc.az_filtered,my_angle.Gyro.x_PI,my_angle.Gyro.y_PI,my_angle.Gyro.z_PI);
                tof_correct(my_angle.Roll,my_angle.Pitch,dl1b_distance_use);}//卡尔曼
//    IMU_Update(my_angle.Gyro.x_PI,my_angle.Gyro.y_PI,my_angle.Gyro.z_PI,my_angle.Acc.x,my_angle.Acc.y,my_angle.Acc.z);//四元数
    
    //-----------------------------数据处理-----------------------------------------
    
    //--------------------------串级内环角速度pid以及并环高度环pid--------------------------------------

    if(!high_I_F_flag)PID_F_Calculate(&HIGH_speed, tof_speed, HIGH_loca.output);//目标值为垂直速度，得做出变量表示垂直速度
    
    if(ASPEED_I_F)
    {
      if(!Aspeed_flag)
      {
        ASPEED_PID_Calculate(&ASPEED_Pitch_con,my_angle.Gyro.y, 0);
        ASPEED_PID_Calculate(&ASPEED_Roll_con,my_angle.Gyro.x , 0);
//        ASPEED_PID_Calculate(&ASPEED_Yaw_con,my_angle.Gyro.z , 0);
      }
      else
      {
        ASPEED_PID_Calculate(&ASPEED_Pitch_con, my_angle.Gyro.y, ANGLE_Pitch_PID.output);
        ASPEED_PID_Calculate(&ASPEED_Roll_con,  my_angle.Gyro.x, ANGLE_Roll_PID.output);
//        ASPEED_PID_Calculate(&ASPEED_Yaw_con,my_angle.Gyro.z , ANGLE_Yaw_PID.output);
      }      
      mon_out.rightup   = base_speed - ASPEED_Pitch_con.output - ASPEED_Roll_con.output  +ASPEED_Yaw_con.output + HIGH_out;
      mon_out.rightdown = base_speed + ASPEED_Pitch_con.output - ASPEED_Roll_con.output  -ASPEED_Yaw_con.output + HIGH_out;
      mon_out.leftup    = base_speed - ASPEED_Pitch_con.output + ASPEED_Roll_con.output  -ASPEED_Yaw_con.output + HIGH_out;
      mon_out.leftdown  = base_speed + ASPEED_Pitch_con.output + ASPEED_Roll_con.output  +ASPEED_Yaw_con.output + HIGH_out;
    }
    else
    {
        if (!Aspeed_flag)//单角速度环
      { 
        ASPEED_PID_Calculate(&ASPEED_Pitch_con,my_angle.Gyro.y, 0);
        ASPEED_PID_Calculate(&ASPEED_Roll_con, my_angle.Gyro.x, 0);  
//        ASPEED_PID_Calculate(&ASPEED_Yaw_con,my_angle.Gyro.z , 0);
      }
      else 
      { 
        
        ASPEED_PID_Calculate(&ASPEED_Pitch_con,my_angle.Gyro.y, ANGLE_Pitch_PID.output);
        ASPEED_PID_Calculate(&ASPEED_Roll_con, my_angle.Gyro.x, ANGLE_Roll_PID.output);
        PID_F_Calculate(&ASPEED_Yaw_con,  my_angle.Gyro.z, ANGLE_Yaw_PID.output);
//        ASPEED_PID_Calculate(&ASPEED_Yaw_con,  my_angle.Gyro.z, ANGLE_Yaw_PID.output);          
      }
      mon_out.rightup   = base_speed - ASPEED_Pitch_con.output - ASPEED_Roll_con.output  +ASPEED_Yaw_con.output + HIGH_out;
      mon_out.rightdown = base_speed + ASPEED_Pitch_con.output - ASPEED_Roll_con.output  -ASPEED_Yaw_con.output + HIGH_out;
      mon_out.leftup    = base_speed - ASPEED_Pitch_con.output + ASPEED_Roll_con.output  -ASPEED_Yaw_con.output + HIGH_out;
      mon_out.leftdown  = base_speed + ASPEED_Pitch_con.output + ASPEED_Roll_con.output  +ASPEED_Yaw_con.output + HIGH_out;
    }
    
   
    //--------------------------串级内环角速度pid以及并环高度环pid-------------------------------------------------
    
    
    //---------------------电机输出限幅处理---------------------------------------------------------------
    mon_out.rightup   = motor_limit(mon_out.rightup  , 800.0f,520.0f);
    mon_out.rightdown = motor_limit(mon_out.rightdown, 800.0f,520.0f);
    mon_out.leftup    = motor_limit(mon_out.leftup   , 800.0f,520.0f);
    mon_out.leftdown  = motor_limit(mon_out.leftdown , 800.0f,520.0f);
    //---------------------电机输出限幅处理------------------------------------------------------------
       
    
//    if(!flag222)
//    {
//      if(encoder1_s<35)
//    {
//      mon_out.rightup  =530;
//      mon_out.rightdown=530;
//      mon_out.leftdown =530;
//      mon_out.leftup   =530;
//      if(encoder1_s<2)
//      {
//        pwm_set_duty(mon_Rup  , 660);
//         
//        pwm_set_duty(mon_Rdown, 0); 
//        pwm_set_duty(mon_Ldown, 0); 
//        pwm_set_duty(mon_Lup  , 0); 
//      }
//      else if(encoder1_s<4)
//      {
//        pwm_set_duty(mon_Rdown  , 660);
//        pwm_set_duty(mon_Rup  , 0);  
//        pwm_set_duty(mon_Ldown, 0); 
//        pwm_set_duty(mon_Lup  , 0);
//      }
//      else if(encoder1_s<6)
//      {
//        pwm_set_duty(mon_Ldown  , 660);
//        pwm_set_duty(mon_Rup  , 0); 
//        pwm_set_duty(mon_Rdown, 0); 
//
//        pwm_set_duty(mon_Lup  , 0);
//      }
//      else if(encoder1_s<8)
//      {
//        pwm_set_duty(mon_Lup  , 660);
//        pwm_set_duty(mon_Rup  , 0); 
//        pwm_set_duty(mon_Rdown, 0); 
//        pwm_set_duty(mon_Ldown, 0); 
//
//      }
//      els
//      {
//         pwm_set_duty(mon_Lup  , 0);
//        pwm_set_duty(mon_Rup  , 0); 
//        pwm_set_duty(mon_Rdown, 0); 
//        pwm_set_duty(mon_Ldown, 0);
//      }
        // pwm_set_duty(mon_Lup  ,660);
        // pwm_set_duty(mon_Rup  ,660); 
        // pwm_set_duty(mon_Rdown,660); 
        // pwm_set_duty(mon_Ldown,660);

//        pwm_set_duty(mon_Rup  , (uint32_t)mon_out.rightup  ); 
//        pwm_set_duty(mon_Rdown, (uint32_t)mon_out.rightdown); 
//        pwm_set_duty(mon_Ldown, (uint32_t)mon_out.leftdown ); 
//        pwm_set_duty(mon_Lup  , (uint32_t)mon_out.leftup   ); 
//    }
//    else if (encoder1_s>=35 && tof_real>250)
//    { 
//      flag111++;
//      
//      pwm_set_duty(mon_Rup ,    (uint32_t)mon_out.rightup  ); 
//      pwm_set_duty(mon_Rdown,   (uint32_t)mon_out.rightdown); 
//      pwm_set_duty(mon_Ldown,   (uint32_t)mon_out.leftdown ); 
//      pwm_set_duty(mon_Lup,     (uint32_t)mon_out.leftup   );
//    }
//    else if (encoder1_s>=40 && tof_real>250)
//    {      
//        flag111++;
//        pwm_set_duty(mon_Rup ,  (uint32_t)mon_out.rightup  -(encoder1_s-39)*10 ); 
//        pwm_set_duty(mon_Rdown, (uint32_t)mon_out.rightdown-(encoder1_s-39)*10 ); 
//        pwm_set_duty(mon_Ldown, (uint32_t)mon_out.leftdown -(encoder1_s-39)*10 ); 
//        pwm_set_duty(mon_Lup,   (uint32_t)mon_out.leftup   -(encoder1_s-39)*10 ); 
//    }
//    else if(encoder1_s>=45&& tof_real>250)
//    {
//      pwm_set_duty(mon_Rup ,  530 ); 
//      pwm_set_duty(mon_Rdown, 530 ); 
//      pwm_set_duty(mon_Ldown, 530 ); 
//      pwm_set_duty(mon_Lup,   530 ); 
//    }
//    else 
//    {
//      flag222=1;
//    }   
//    }
//    else
//    {
//        pwm_set_duty(TCPWM_CH28_P10_0, 0); 
//        pwm_set_duty(TCPWM_CH29_P10_1, 0); 
//        pwm_set_duty(TCPWM_CH48_P14_0, 0); 
//        pwm_set_duty(TCPWM_CH49_P14_1, 0);
//    }
//    }

  if(flag111 && tof_real<300)
  {
    pwm_set_duty(mon_Rup  , 0); 
    pwm_set_duty(mon_Rdown, 0); 
    pwm_set_duty(mon_Ldown, 0); 
    pwm_set_duty(mon_Lup  , 0);
    flag222=1;
  }
  else if(!flag222)
  {
     pwm_set_duty(mon_Rup  , (uint32_t)mon_out.rightup  ); 
     pwm_set_duty(mon_Rdown, (uint32_t)mon_out.rightdown); 
     pwm_set_duty(mon_Ldown, (uint32_t)mon_out.leftdown ); 
     pwm_set_duty(mon_Lup  , (uint32_t)mon_out.leftup   );
  }
    

}

void pit0_ch1_isr()                     // 定时器通道 1 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH1);
    //计时
    static int High_set=50;
    if (++encoder10_ms == 100) 
    {

               encoder1_s++;
               encoder10_ms = 0;
    }
    if(encoder1_s==3)
    {
        flagqingling=1;
    }
    if(kf_ekf)
    {
      if(!locat_dis)
      {
        PID_F_Calculate(&ANGLE_Pitch_PID,angle_ekf.Pitch, 0);
        PID_F_Calculate(&ANGLE_Roll_PID,angle_ekf.Roll, 0);
      }
      else
      {
        PID_F_Calculate(&ANGLE_Pitch_PID,angle_ekf.Pitch, Locat_y_PID.output);
        PID_F_Calculate(&ANGLE_Roll_PID,angle_ekf.Roll,  -Locat_x_PID.output);
      }      
      PID_F_Calculate(&ANGLE_Yaw_PID,my_angle.Yaw, Yaw_target);//右转是负值
    }
    else
    {
      if(!locat_dis )//加一个tof_real < 700，让高度环下降时先不找灯
      {
        PID_F_Calculate(&ANGLE_Pitch_PID,my_angle.Pitch, 0);
        PID_F_Calculate(&ANGLE_Roll_PID, my_angle.Roll, 0);  
      }
      else
      {

        PID_F_Calculate(&ANGLE_Pitch_PID,my_angle.Pitch, Locat_y_PID.output);
        PID_F_Calculate(&ANGLE_Roll_PID, my_angle.Roll,  -Locat_x_PID.output);
      }      
      PID_F_Calculate(&ANGLE_Yaw_PID,my_angle.Yaw, Yaw_target);
    }  
    if(high_flag) 
    {
      if(high_I_F_flag)
      {
        PID_I_Calculate(&HIGH_PID, tof_real, High);
      if(tof_real<200 && HIGH_PID.output<0) HIGH_PID.output=0;
      }
      else
      {
        if(!flag111) High_set+=2;
        else High_set-=2;
        if(High_set>High) High_set=High;//做一个缓启动
        if(High_set<300  && flag111) High_set=0;
       PID_F_Calculate(&HIGH_loca, tof_real, High_set);//高度串速度
       High_target =High_set;
      }
    }  
}


void pit0_ch2_isr()                     // 定时器通道 2 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH2);
    pmw_dataget();

    PID_F_Calculate(&Locat_x_PID,pmw3901_delta_x_use, 0);//向左为x正 ,roll为负，目前逻辑应该是向roll轴直接输入目标值为pmw3901_delta_x即可 
    PID_F_Calculate(&Locat_y_PID,pmw3901_delta_y_use, imageYerr);//向上为y正，pitch为正，目前逻辑应该是向roll轴直接输入目标值为-pmw3901_delta_y即可 
    if(imageYerr != 0.0f)
    {
        if(Locat_y_PID.output > 5.0f) Locat_y_PID.output =5.0f;
        else if(pmw3901_delta_y_use<0.5f && Locat_y_PID.output<3.0f) Locat_y_PID.output=3.0f;
        else if(Locat_y_PID.output < 0.0f) Locat_y_PID.output = 0.0f;
    }
    //将y方向输入从0改为imageYerr；
    //后续在上述函数中传入摄像头获取的目标点坐标误差即可
    
}

void pit0_ch10_isr()                    // 定时器通道 10 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH10);
    
}

void pit0_ch11_isr()                    // 定时器通道 11 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH11);
    
}

void pit0_ch12_isr()                    // 定时器通道 12 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH12);
    
}

void pit0_ch13_isr()                    // 定时器通道 13 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH13);
    
}

void pit0_ch14_isr()                    // 定时器通道 14 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH14);
    
}

void pit0_ch15_isr()                    // 定时器通道 15 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH15);
    
}

void pit0_ch16_isr()                    // 定时器通道 16 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH16);
    
}

void pit0_ch17_isr()                    // 定时器通道 17 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH17);
    
}

void pit0_ch18_isr()                    // 定时器通道 18 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH18);
    
}

void pit0_ch19_isr()                    // 定时器通道 19 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH19);
    
}

void pit0_ch20_isr()                    // 定时器通道 20 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH20);
    
}

void pit0_ch21_isr()                    // 定时器通道 21 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH21);
    tsl1401_collect_pit_handler();
}
// **************************** PIT中断函数 ****************************


// **************************** 串口中断函数 ****************************
// 串口0默认作为调试串口
void uart0_isr (void)
{
    if(uart_isr_mask(UART_0))            // 串口0接收中断
    {
        
#if DEBUG_UART_USE_INTERRUPT             // 如果开启 debug 串口中断
        debug_interrupr_handler();       // 调用 debug 串口接收处理函数 数据会被 debug 环形缓冲区读取
#endif                                   // 如果修改了 DEBUG_UART_INDEX 那这段代码需要放到对应的串口中断去
      
    }
    else                                 // 串口0发送中断
    {           
        
        
        
    }
}

void uart1_isr (void)
{
    if(uart_isr_mask(UART_1))            // 串口1接收中断
    {
        
        wireless_module_uart_handler();  // 无线模块统一回调函数
      
    }
    else                                // 串口1发送中断
    {
      
        
        
    }
}

void uart2_isr (void)
{
    if(uart_isr_mask(UART_2))            // 串口2接收中断
    {
        
        gnss_uart_callback();            // GPS模块回调函数      
        
    }
    else                                // 串口2发送中断
    {
        
        
       
    }
}

void uart3_isr (void)
{
    if(uart_isr_mask(UART_3))            // 串口3接收中断
    {
        
        
        
    }
    else                                // 串口3发送中断
    {
      
        
        
    }
}

void uart4_isr (void)
{
    if(uart_isr_mask(UART_4))            // 串口4接收中断
    {

        uart_receiver_handler();                                                                // 串口接收机回调函数
       
    }
    else                                // 串口4发送中断
    {
      
        
        
    }
}

void uart5_isr (void)
{
    if(uart_isr_mask(UART_5))            // 串口5接收中断
    {
        
        
       
    }
    else                                // 串口5发送中断
    {
      
        
        
    }
}

void uart6_isr (void)
{
    if(uart_isr_mask(UART_6))            // 串口6接收中断
    {

        
       
    }
    else                                // 串口6发送中断
    {
      
        
        
    }
}
// **************************** 串口中断函数 ****************************

// **************************** 外部中断函数 ****************************
void gpio_0_exti_isr()                  // 外部 GPIO_0 中断服务函数     
{
    
  
  
}

void gpio_1_exti_isr()                  // 外部 GPIO_1 中断服务函数     
{
    if(exti_flag_get(P01_0))		// 示例P1_0端口外部中断判断
    {

      
      
            
    }
    if(exti_flag_get(P01_1))
    {

            
            
    }
}

void gpio_2_exti_isr()                  // 外部 GPIO_2 中断服务函数     
{
    if(exti_flag_get(P02_0))
    {
            
            
    }
    if(exti_flag_get(P02_4))
    {
            
            
    }

}

void gpio_3_exti_isr()                  // 外部 GPIO_3 中断服务函数     
{



}

void gpio_4_exti_isr()                  // 外部 GPIO_4 中断服务函数     
{



}

void gpio_5_exti_isr()                  // 外部 GPIO_5 中断服务函数     
{



}


void gpio_6_exti_isr()                  // 外部 GPIO_6 中断服务函数     
{



}

void gpio_7_exti_isr()                  // 外部 GPIO_7 中断服务函数     
{



}

void gpio_8_exti_isr()                  // 外部 GPIO_8 中断服务函数     
{



}

void gpio_9_exti_isr()                  // 外部 GPIO_9 中断服务函数     
{



}

void gpio_10_exti_isr()                  // 外部 GPIO_10 中断服务函数     
{



}

void gpio_11_exti_isr()                  // 外部 GPIO_11 中断服务函数     
{



}

void gpio_12_exti_isr()                  // 外部 GPIO_12 中断服务函数     
{



}

void gpio_13_exti_isr()                  // 外部 GPIO_13 中断服务函数     
{



}

void gpio_14_exti_isr()                  // 外部 GPIO_14 中断服务函数     
{



}

void gpio_15_exti_isr()                  // 外部 GPIO_15 中断服务函数     
{



}

void gpio_16_exti_isr()                  // 外部 GPIO_16 中断服务函数     
{



}

void gpio_17_exti_isr()                  // 外部 GPIO_17 中断服务函数     
{



}

void gpio_18_exti_isr()                  // 外部 GPIO_18 中断服务函数     
{



}

void gpio_19_exti_isr()                  // 外部 GPIO_19 中断服务函数     
{



}

void gpio_20_exti_isr()                  // 外部 GPIO_20 中断服务函数     
{



}

void gpio_21_exti_isr()                  // 外部 GPIO_21 中断服务函数     
{



}

void gpio_22_exti_isr()                  // 外部 GPIO_22 中断服务函数     
{



}

void gpio_23_exti_isr()                  // 外部 GPIO_23 中断服务函数     
{



}
// **************************** 外部中断函数 ****************************
