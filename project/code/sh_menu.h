#ifndef CODE_SH_MENU_H_
#define CODE_SH_MENU_H_

#include "zf_common_typedef.h"

#define key1 (P20_3)
#define key2 (P20_2)
#define key3 (P20_1)
#define key4 (P20_0)

#define DIP_IO_4 (P13_2)
#define DIP_IO_3 (P13_1)
#define DIP_IO_2 (P13_4)
#define DIP_IO_1 (P13_3)

extern uint8 keynum;
extern uint8 keynum1;

void DIP_init(void);
void Key_Init(void);
uint8 Key_Scan(void);
//void sh_menu(void);


#endif 