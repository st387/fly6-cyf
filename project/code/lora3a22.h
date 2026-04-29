#ifndef _lora3a22_h_
#define _lora3a22_h_

#include "zf_common_typedef.h"


#define LORA3A22_UART_INDEX            (UART_1)              // 定义串口遥控器使用的串口
#define LORA3A22_UART_TX_PIN           (UART1_TX_P04_1)      // 遥控器接收机的RX引脚 连接单片机的TX引脚
#define LORA3A22_UART_RX_PIN           (UART1_RX_P04_0)      // 遥控器接收机的TX引脚 连接单片机的RX引脚
#define LORA3A22_UART_BAUDRATE         (115200)              // 指定 lora3a22 串口所使用的的串口波特率

#define LORA3A22_DATA_LEN              ( 12  )               // lora3a22帧长
#define LORA3A22_FRAME_STAR            ( 0XA3 )              // 帧头信息


typedef struct
{
    uint8 head;                                             // 帧头
    uint8 sum_check;                                        // 和校验
    uint8 key[2];                                           // 摇杆按键    左边:key[0]   右边:key[1]   按下0 松开1
                                                              
    int16 joystick[4];                                      //joystick[0]:左边摇杆左右值      joystick[1]:左边摇杆上下值
                                                            //joystick[2]:右边摇杆左右值      joystick[3]:右边摇杆上下值
                                                            //摇杆都是右下为正
}lora3a22_uart_transfer_dat_struct ;

extern lora3a22_uart_transfer_dat_struct lora3a22_uart_transfer;
extern uint8   lora3a22_uart_data[LORA3A22_DATA_LEN];       // lora3a22接收原始数据
extern vuint8  lora3a22_finsh_flag;
extern vuint8  lora3a22_state_flag;                         // 遥控器状态(1表示正常，否则表示失控)
extern uint16  lora3a22_response_time;

void lora3a22_init(void);

#endif
