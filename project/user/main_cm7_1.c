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
* 文件名称          main_cm7_1
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT4BB
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-1-4       pudding            first version
*********************************************************************************************************************/

#include "zf_common_headfile.h"
// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
#define DATA_LENGTH               (6)                                           // 数组数据长度

#pragma location = 0x28001000                                                   // 将下面这个数组定义到指定的RAM地址，便于其他核心直接访问(开源库默认在 0x28001000 地址保留了8kb的空间用于数据交互)
                                                                                // 此处为0x28001014的原因是前面放了一个M0的数组
float m7_1_data[DATA_LENGTH] = {0, 0, 0, 0, 0, 0};                             // 定义 M7_1 演示数据数组 浮点数类型

#pragma location = 0x28080ab0

float image_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
float loradata_To0[DATA_LENGTH] = {0, 0, 0, 0, 0, 0};
// **************************** 代码区域 ****************************
float avg_x, avg_y;
float beacon_x, beacon_y;
int tft_state2=0;
int main(void)
{
    clock_init(SYSTEM_CLOCK_250M);     // 时钟配置及系统初始化<务必保留>
    debug_info_init();                 // 调试串口信息初始化

    // 此处编写用户代码 例如外设初始化代码等
    lora3a22_init();
    mt9v03x_init();
//    tft180_init();
    gpio_init(DIP_IO_3, GPI, 1, GPI_PULL_UP);
    if(gpio_get_level(DIP_IO_3))
    {
      tft180_init();
      tft_state2=gpio_get_level(DIP_IO_3);
    }
    pit_ms_init(PIT_CH10, 10);

    // 此处编写用户代码 例如外设初始化代码等
    while (true)
    {
        // 此处编写需要循环执行的代码
        if (lora3a22_state_flag == 1)
        {
            if (lora3a22_finsh_flag == 1)
            {
//                //lora3a22 帧头
////                printf ("head = %d\r\n",lora3a22_uart_transfer.head);
////
////                //lora3a22 和校验
////                printf ("sum_check = %d\r\n",lora3a22_uart_transfer.sum_check);
////
////                //左边摇杆左右值
////                printf ("joystick[0] = %d\r\n",lora3a22_uart_transfer.joystick[0]);
////                //左边摇杆上下值
////                printf ("joystick[1] = %d\r\n",lora3a22_uart_transfer.joystick[1]);
////                //右边摇杆左右值
////                printf ("joystick[2] = %d\r\n",lora3a22_uart_transfer.joystick[2]);
////                //右边摇杆上下值
////                printf ("joystick[3] = %d\r\n",lora3a22_uart_transfer.joystick[3]);
////
////                //左边摇杆按键
////                printf ("key0 = %d\r\n",lora3a22_uart_transfer.key[0]);
////                //右边摇杆按键
////                printf ("key1 = %d\r\n",lora3a22_uart_transfer.key[1]);
                m7_1_data[0] = lora3a22_uart_transfer.key[0];
                m7_1_data[1] = lora3a22_uart_transfer.key[1];
                m7_1_data[2] = lora3a22_uart_transfer.joystick[0];
                m7_1_data[3] = lora3a22_uart_transfer.joystick[1];
                m7_1_data[4] = lora3a22_uart_transfer.joystick[2];
                m7_1_data[5] = lora3a22_uart_transfer.joystick[3];
                lora3a22_finsh_flag = 0;
            }
        }
        else
        {
            memcpy(&m7_1_data, &loradata_To0, sizeof(m7_1_data));
        }
        SCB_CleanInvalidateDCache_by_Addr(&m7_1_data, sizeof(m7_1_data)); // M7_1核心有Dcache 当数据有变化时应该更新Dcache的内容 否则数据无法同步到RAM(其他核心访问的RAM地址也就无法读取到数据)

        if (mt9v03x_finish_flag)
        {
            sobel(mt9v03x_image[0], image_sobel[0], MT9V03X_W, MT9V03X_H, 120);
            if (two_pass(image_sobel, &max_point, &avg_x, &avg_y))
            {
                find_real_beacons(&beacon_x, &beacon_y);
            }
            if(tft_state2)
            {
                tft180_show_gray_image(0, 0, beacon_only[0], MT9V03X_W, MT9V03X_H, MT9V03X_W / 2, MT9V03X_H / 2, 0);
                tft180_show_float(0, 110, beacon_x, 3, 2);
                tft180_show_float(50, 110, beacon_y, 3, 2);
                tft180_draw_point(beacon_x / 2, beacon_y / 2, RGB565_RED);
                tft180_show_float(0,120,lora3a22_uart_transfer.joystick[0],3,3);
                tft180_show_float(45,120,lora3a22_uart_transfer.joystick[1],3,1);
             }
            
            image_data[0] = beacon_x;
            image_data[1] = beacon_y;
            SCB_CleanInvalidateDCache_by_Addr(&image_data, sizeof(image_data));
            mt9v03x_finish_flag = 0;
        }

        // 此处编写需要循环执行的代码
    }
}

// **************************** 代码区域 ****************************