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
* 文件名称          main_cm7_0
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT4BB
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-1-4       pudding            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
#define DATA_LENGTH               (6)                    // 数组数据长度

#pragma location = 0x28001000                            // 将下面这个数组定义到指定的RAM地址，#pragma需要手动分配地址，因此需要计算数据长度后再分配
__no_init float m7_1_data[DATA_LENGTH];                 // 定义M7_1演示数据数组 浮点数类型  由于该数组已经在M7_1核心赋值过初值，因此此处不再初始化
 


#pragma location = 0x28080ab0
__no_init float image_data[8];

int tft_state=0;

// ===== 新增：面向灯判断与图像y误差滤波 =====
int yaw_align_ok = 0;
uint8 image_slow_add_cnt = 0;
int yaw_reset_done = 0;
float imageYerr_filtered = 0.0f;
float imagePitchTarget = 0.0f;
float imagePitchTargetCmd = 0.0f;
uint8 image_find_state = 0;
uint8 image_align_stable_cnt = 0;
uint8 image_target_lost_cnt = 0;



#if 0
#define X_ALIGN_TH        8.0f
#define IMAGE_Y_ALPHA     0.2f
#define FIND_SEARCH       0
#define FIND_ALIGN_X      1
#define FIND_APPROACH     2
#define FIND_STOP         3
#define IMAGE_X_CENTER    70.0f
#define IMAGE_Y_STOP_LINE 105.0f
#define IMAGE_Y_SLOW_LINE 90.0f
#define IMAGE_Y_MID_LINE  75.0f
#define IMAGE_PITCH_FAST  3.0f
#define IMAGE_PITCH_MID   2.0f
#define IMAGE_PITCH_SLOW  1.0f
#define YAW_SEARCH_TARGET 25.0f
#endif

#if 0
#define X_ALIGN_TH               8.0f
#define FIND_SEARCH              0
#define FIND_ALIGN_X             1
#define FIND_APPROACH            2
#define FIND_STOP                3
#define IMAGE_X_CENTER           70.0f
#define IMAGE_Y_STOP_LINE        94.0f
#define IMAGE_Y_SLOW_LINE        76.0f
#define IMAGE_Y_MID_LINE         62.0f
#define IMAGE_PITCH_FAST         1.8f
#define IMAGE_PITCH_MID          0.70f
#define IMAGE_PITCH_SLOW         0.25f
#define IMAGE_ALIGN_OK_CNT       3
#define IMAGE_TARGET_LOST_HOLD_CNT 1
#define IMAGE_PITCH_ALIGN_HOLD   0.15f
#define IMAGE_PITCH_LOST_HOLD    0.12f
#define IMAGE_PITCH_UP_ALPHA     0.10f
#define IMAGE_PITCH_DOWN_ALPHA   0.55f
#define IMAGE_PITCH_DEADBAND     0.01f
#define YAW_SEARCH_TARGET        40.0f
#endif

enum
{
    FIND_SEARCH = 0,
    FIND_ALIGN_X = 1,
    FIND_APPROACH = 2,
    FIND_STOP = 3
};

// **************************** 代码区域 ****************************
int main(void)
{
  
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化<务必保留>
    debug_init();                       // 调试串口信息初始化
    // 此处编写用户代码 例如外设初始化代码等
    
//     gpio_init(P22_1, GPO, 1, GPO_PUSH_PULL); 
    
//    system_delay_ms(1000);
//    pwm_set_duty(mon_Rup ,  530);
//    pwm_set_duty(mon_Rdown, 530);
//    pwm_set_duty(mon_Ldown, 530);
//    pwm_set_duty(mon_Lup,   530);   
    dl1b_init();
    imu660ra_init();
    Err_Reduce();//去零漂
//    icm42688_init(&ICM42688_CONFIG);
//    icm42688_Err_Reduce();
    pmw3901_init();
    motor_init();
//    pmw_err_init();
//    tft180_set_dir(TFT180_PORTAIT_180);
    gpio_init(DIP_IO_1, GPI, 1, GPI_PULL_UP);
    if(gpio_get_level(DIP_IO_1))
    {
      tft180_init();
      tft_state=gpio_get_level(DIP_IO_1);
    }
    
    
//    mt9v03x_init();
    
//    lora3a22_init();
//    DIP_init();
    
//    gpio_init(DIP_IO_3, GPI, 1, GPI_PULL_UP);
//    Key_Init();
    
//    PID_F_Init(&ANGLE_Pitch_PID,  3.3f, 0.001f, 0.339f, 100, 1000);//2.30//3.3，0.001，0.339，100，1000
//    PID_F_Init(&ANGLE_Roll_PID,   3.3f, 0.001f, 0.339f, 100, 1000);
    PID_F_Init(&ANGLE_Pitch_PID,  15.3f, 0.0000f, 0.0f, 100, 1000);//13.3,0,25
    PID_F_Init(&ANGLE_Roll_PID,   15.3f, 0.0000f, 0.0f, 100, 1000);
    PID_F_Init(&ANGLE_Yaw_PID,    16.3f, 0.0000f, 0.0f, 100, 1000);
    //角速度环不能太硬，油门问题串级输出做缩放
    //---------------------------位置式角速度----------------------
    PID_F_Init(&ASPEEDF_Pitch_PID,  0.01f, 0.000005f, 0.f, 100, 1000);//2.30//3.3，0.001，0.339，100，1000
    PID_F_Init(&ASPEEDF_Roll_PID,   0.01f, 0.000005f, 0.f, 100, 1000);
    PID_F_Init(&ASPEEDF_Yaw_PID,    0.04f, 0.00000f,  0.f, 100, 30);
    //---------------------------位置式角速度----------------------
    //---------------------------增量式角速度----------------------
    PID_I_Init(&ASPEED_Pitch_PID,  0.005f, 0.0001f, 0.00f, 40, 170);//0.09，0.0005，0.45速620    //0.05,0.0006,0.1//以上全是3s版本
    PID_I_Init(&ASPEED_Roll_PID,   0.005f, 0.0001f, 0.00f, 40, 170);                             
    PID_I_Init(&ASPEED_Yaw_PID,    0.005f, 0.0001f, 0.00f, 40, 170);
    //---------------------------增量式角速度----------------------
    //---------------------------高度---------------------//0.005，0.00006，0.06//目前测出来pd都偏大
    PID_I_Init(&HIGH_PID, 0.004f , 0.00003f, 0.0f, 8, 8);//0.005,0.00005,0.08,8,8,这套参数因为d项参数偏大，因此会在刚上电时出现负值
    PID_F_Init(&HIGH_speed, 3.8f , 0.0000f,  0.2f, 6, 13);
    PID_F_Init(&HIGH_loca, 0.015f , 0.0000f, 0.0f, 6, 8);
    //---------------------------高度---------------------
    //---------------------------位置环---------------------
    PID_F_Init(&Locat_x_PID,11.8f, 0, 0, 0, 20);
    PID_F_Init(&Locat_y_PID,11.8f, 0, 0, 0, 20);
    //---------------------------位置---------------------

    pit_ms_init(PIT_CH0, 3);//角速度，高度5ms
    pit_ms_init(PIT_CH1, 10);//角度10ms
    pit_ms_init(PIT_CH2, 20);
//    interrupt_set_priority//设置中断优先级函数
    
     // 此处编写用户代码 例如外设初始化代码等
    while(true)
    {

      SCB_CleanInvalidateDCache_by_Addr(&m7_1_data, sizeof(m7_1_data)); //数据沟通api
      SCB_CleanInvalidateDCache_by_Addr(&image_data, sizeof(image_data));//图像
            // ===== 原逻辑保留
//      image_x=image_data[0];
//      image_y=image_data[1];
//      if(image_x>0 && image_y>0&&tof_real>800)
//      {
//        Yaw_target=(70-image_x)*1.6;
//      }
//      else if(image_x<0 && image_y<0&&tof_real>800)
//      {
//        Yaw_target=50;
//      }
//      else Yaw_target=0;
//      if(image_x>65 && image_x<75 && image_y>0 )
//      {
//        imageYerr=(100-image_y)*0.03;
//        if(imageYerr>0.5) imageYerr=0.5;
//        else if(imageYerr<-0.5) imageYerr=-0.5;
//        if(100-image_y<5 && 100-image_y>-5) imageYerr=0;
////        imageYerr=(100-image_y)*0.1;
////        if(imageYerr>1) imageYerr=1;
////        else if(imageYerr<-1) imageYerr=-1;
////        if(imageYerr<0.5 && imageYerr>-0.5) imageYerr=0;
//      }
//      else
//      {
//        imageYerr=0;
//      }


  
      image_x = image_data[0];
      image_y = image_data[1];
      
//      if(image_x < 0 && image_y < 0 && image_L_y<30.0f)
//      {
//        image_x=image_L_x;
//        image_y=image_L_y;
//      }
        
      if (image_x > 0 && image_y > 0 && tof_real > 700)
      {
        Yaw_target = (75.0f - image_x) * 1.6f;
      }
      else if (image_x < 0 && image_y < 0 && tof_real > 700)
      {
        Yaw_target = 50.0f;
      }
      else
      {
        Yaw_target = 0.0f;
      }

      if (image_x > 0 && image_y > 0 && fabsf(75.0f - image_x) < 20.0f)
      {
        yaw_align_ok = 1;
      }
      else
      {
        yaw_align_ok = 0;
      }

      if (yaw_align_ok && image_y > 0)
      {
          float y_err = 110.0f - image_y;
          float imageYerr_raw = 0.0f;
          if (y_err > 40.0f)
          {
            image_slow_add_cnt=0;
            imageYerr_raw=1.0f;
//              imageYerr_raw = y_err * 0.05f;
//
//              if (imageYerr_raw > 0.8f) imageYerr_raw = 0.8f;
//              else if (imageYerr_raw < -0.8f) imageYerr_raw = -0.8f;
//              if (pmw3901_delta_y_use > 1.0f)   
//              {
//                imageYerr_raw = -0.12f;
//              }
//              else if (pmw3901_delta_y_use > 0.6f)   
//              {
//                imageYerr_raw = 0.0f;
//              }
          }
          else
          {
              image_slow_add_cnt++;
              if(image_slow_add_cnt > 200 )image_slow_add_cnt=200;
              imageYerr_raw = y_err * 0.05f;
  
              if(image_slow_add_cnt>100)
              {
                if (y_err <20.0f) image_slow_add_cnt=0;
                if (imageYerr_raw > 0.8f) imageYerr_raw = 0.8f;
                else if (imageYerr_raw < -0.8f) imageYerr_raw = -0.8f;
//                if (pmw3901_delta_y_use > 1.0f)   
//                {
//                  imageYerr_raw = -0.12f;
//                }
//                else if (pmw3901_delta_y_use > 0.6f)   
//                {
//                  imageYerr_raw = 0.0f;
//                }
              }
              else
              {
                if (imageYerr_raw > 0.3f) imageYerr_raw = 0.3f;
                else if (imageYerr_raw < -0.3f) imageYerr_raw = -0.3f;
//                if (pmw3901_delta_y_use > 0.5f)   
//                {
//                  imageYerr_raw = -0.12f;
//                }
//                else if (pmw3901_delta_y_use > 0.3f)   
//                {
//                  imageYerr_raw = 0.0f;
//                }
              }
          }
          imageYerr_filtered = 0.2 * imageYerr_raw +
                               (1.0f - 0.2) * imageYerr_filtered;
          imageYerr = imageYerr_filtered;
      }
      else
      {
          imageYerr_filtered = 0.0f;
          imageYerr = 0.0f;
      }



      


#if 0
      if (image_x > 0 && image_y > 0 && tof_real > 700)
      {
          float x_err = IMAGE_X_CENTER - image_x;
          image_target_lost_cnt = 0;

          Yaw_target = x_err * 1.6f;

          if (fabsf(x_err) < X_ALIGN_TH)
          {
              if (image_align_stable_cnt < 250)
              {
                  image_align_stable_cnt++;
              }
          }
          else
          {
              image_align_stable_cnt = 0;
          }

          yaw_align_ok = (image_align_stable_cnt >= IMAGE_ALIGN_OK_CNT);

          if (!yaw_align_ok)
          {
              image_find_state = FIND_ALIGN_X;
              imagePitchTargetCmd = IMAGE_PITCH_ALIGN_HOLD;
          }
          else if (image_y >= IMAGE_Y_STOP_LINE)
          {
              image_find_state = FIND_STOP;
              imagePitchTargetCmd = 0.0f;
          }
          else if (image_y >= IMAGE_Y_SLOW_LINE)
          {
              image_find_state = FIND_APPROACH;
              imagePitchTargetCmd = IMAGE_PITCH_SLOW;
          }
          else if (image_y >= IMAGE_Y_MID_LINE)
          {
              image_find_state = FIND_APPROACH;
              imagePitchTargetCmd = IMAGE_PITCH_MID;
          }
          else
          {
              image_find_state = FIND_APPROACH;
              imagePitchTargetCmd = IMAGE_PITCH_FAST;
          }
      }
      else
      {
          yaw_align_ok = 0;
          if (image_find_state != FIND_SEARCH &&
              image_target_lost_cnt < IMAGE_TARGET_LOST_HOLD_CNT)
          {
              image_target_lost_cnt++;
              image_find_state = FIND_ALIGN_X;
              Yaw_target = 0.0f;
              imagePitchTargetCmd = IMAGE_PITCH_LOST_HOLD;
          }
          else
          {
              image_align_stable_cnt = 0;
              image_target_lost_cnt = 0;
              image_find_state = FIND_SEARCH;

              if (tof_real > 700)
              {
                  Yaw_target = YAW_SEARCH_TARGET;
              }
              else
              {
                  Yaw_target = 0.0f;
              }

              imagePitchTargetCmd = 0.0f;
          }
      }

      if (imagePitchTargetCmd > imagePitchTarget)
      {
          imagePitchTarget += (imagePitchTargetCmd - imagePitchTarget) * IMAGE_PITCH_UP_ALPHA;
      }
      else
      {
          imagePitchTarget += (imagePitchTargetCmd - imagePitchTarget) * IMAGE_PITCH_DOWN_ALPHA;
      }

      if (fabsf(imagePitchTarget) < IMAGE_PITCH_DEADBAND)
      {
          imagePitchTarget = 0.0f;
      }

      imageYerr_filtered = imagePitchTargetCmd;
      imageYerr = imagePitchTarget;
#endif
      
      image_L_x=image_x;
      image_L_y=image_y;
      if(tft_state)
      {
//        tft180_show_float(0,0, pmw3901_delta_x_use,5,3);
//        tft180_show_float(0,15,pmw3901_delta_y_use,5,3);                 tft180_show_float(45, 15,my_angle.Pitch,5,3);
//        tft180_show_float(0,30,pmw3901_delta_x_i , 5,3);                 tft180_show_float(45, 30,my_angle.Roll,5,3);
        tft180_show_float(0,45,image_x    , 5,3);                        tft180_show_float(45,45,yaw_align_ok  , 5,3);
        tft180_show_float(0,60,image_y  , 5,3);                          tft180_show_float(45,60,ANGLE_Pitch_PID.output  , 5,3);  
        tft180_show_float(0,74,dl1b_distance_mm,5,3);                    tft180_show_float(45,74,imageYerr,5,3);
        tft180_show_float(0 ,90,tof_real ,5,3);                         tft180_show_float(45 ,90,Locat_y_PID.output ,5,3);
      }
        
//        printf("%f,%f,%f,%f,%f\n",my_angle.Gyro.y,my_angle.Gyro.x,my_angle.Pitch,my_angle.Roll,my_angle.Yaw);
//        if(m7_1_data[0])
//        {
//            if(m7_1_data[5]>1500) 
//          {
//            if(m7_1_data[5]>Yaw_target) Yaw_target=60;
//          }
//          else if (m7_1_data[5]<-1500)
//          {
//            if(m7_1_data[5]<Yaw_target) Yaw_target=-60;
//          }
//        }

        if(!m7_1_data[1])
        {
          encoder1_s=35;
        }
        if(!m7_1_data[0])
        {
          encoder1_s=40;
        }
        if(!m7_1_data[1]||!m7_1_data[0])
        {
          flag111=1;
        }
      
      
    }
}

// **************************** 代码区域 ****************************
