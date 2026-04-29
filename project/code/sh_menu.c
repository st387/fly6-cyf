#if 0
#include "zf_common_headfile.h"



struct linknode
{
        int data;
        struct linknode* next;
        struct linknode* last;
        struct linknode* nextlev;
        struct linknode* lastlev;

};

uint8 keynum;
uint8 keynum1;

//拨码初始化
void DIP_init(void)
{
    gpio_init(DIP_IO_1, GPI, 1, GPI_PULL_UP);
    gpio_init(DIP_IO_2, GPI, 1, GPI_PULL_UP);
    gpio_init(DIP_IO_3, GPI, 1, GPI_PULL_UP);
    gpio_init(DIP_IO_4, GPI, 1, GPI_PULL_UP);
}

void Key_Init(void)
{
    key_init(10);
    gpio_init(key1, GPI, 1, GPI_PULL_UP);
    gpio_init(key2, GPI, 1, GPI_PULL_UP);
    gpio_init(key3, GPI, 1, GPI_PULL_UP);
    gpio_init(key4, GPI, 1, GPI_PULL_UP);
}

uint8 Key_Scan(void)
{
    key_scanner();
    if(key_get_state(KEY_1)==KEY_SHORT_PRESS)
        return 1;
    else if(key_get_state(KEY_2)==KEY_SHORT_PRESS)
        return 2;
    else if(key_get_state(KEY_3)==KEY_SHORT_PRESS)
        return 3;
    else if(key_get_state(KEY_4)==KEY_SHORT_PRESS)
        return 4;

    else if(key_get_state(KEY_1)==KEY_LONG_PRESS)
    {
        system_delay_ms(500);
        return 11;
    }
    else if(key_get_state(KEY_2)==KEY_LONG_PRESS)
    {
        system_delay_ms(500);
        return 22;
    }
    else if(key_get_state(KEY_3)==KEY_LONG_PRESS)
    {
        system_delay_ms(500);
        return 33;
    }
    else if(key_get_state(KEY_4)==KEY_LONG_PRESS)
    {
        system_delay_ms(500);
        return 44;
    }
    else
    {
        return 0;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     菜单使用函数
// 返回参数     void
// 返回参数     void
// 使用示例     sh_menu();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void sh_menu(void)
{
    //先创建各级菜单节点，节点名和数据可根据后续具体功能更改
    //创建一级菜单
    struct linknode menu1 = {1,NULL,NULL,NULL,NULL};
    struct linknode menu2 = {2,NULL,NULL,NULL,NULL};
    struct linknode menu3 = {3,NULL,NULL,NULL,NULL};
    struct linknode menu4 = {4,NULL,NULL,NULL,NULL};

    //创建二级菜单
    struct linknode menu11 = {11,NULL,NULL,NULL,NULL};
    struct linknode menu12 = {12,NULL,NULL,NULL,NULL};
    struct linknode menu13 = {13,NULL,NULL,NULL,NULL};
    struct linknode menu14 = {14,NULL,NULL,NULL,NULL};

    //创建三级菜单
    struct linknode menu111 = {111,NULL,NULL,NULL,NULL};
    struct linknode menu112 = {112,NULL,NULL,NULL,NULL};
    struct linknode menu113 = {113,NULL,NULL,NULL,NULL};
    struct linknode menu114 = {114,NULL,NULL,NULL,NULL};
    //在某一级菜单增加节点，在对应位置创建节点即可
    //再创建更多级菜单，参照上述创建即可
    //创建后在下方对节点进行初步链接
    
    //完善一级菜单的前后节点指针，以及跳下一级菜单
    menu1.next = &menu2;
    menu1.last = &menu4;
    menu1.nextlev = &menu11;

    menu2.next = &menu3;
    menu2.last = &menu1;
    menu2.nextlev = &menu11;

    menu3.next = &menu4;
    menu3.last = &menu1;
    menu3.nextlev = &menu11;

    menu4.next = &menu1;
    menu4.last = &menu3;
    menu4.nextlev = &menu11;

    //完善二级菜单的前后节点指针，以及跳下级菜单,这里跳上级菜单需要在上级跳下级时进行设置，让下级能跳回原来的上级，而不是上级菜单的第一节点
    menu11.next = &menu12;
    menu11.last = &menu14;
    menu11.nextlev = &menu111;

    menu12.next = &menu13;
    menu12.last = &menu11;
    menu12.nextlev = &menu111;

    menu13.next = &menu14;
    menu13.last = &menu12;
    menu13.nextlev = &menu111;

    menu14.next = &menu11;
    menu14.last = &menu13;
    menu14.nextlev = &menu111;

    //完善三级菜单的前后节点指针，这里跳上级菜单需要在上级跳下级时进行设置，让下级能跳回原来的上级，而不是上级菜单的第一节点
    menu111.next = &menu112;
    menu111.last = &menu114;

    menu112.next = &menu113;
    menu112.last = &menu111;

    menu113.next = &menu114;
    menu113.last = &menu112;

    menu114.next = &menu111;
    menu114.last = &menu113;

    struct linknode* pc = &menu1;
    struct linknode* last_pc=&menu1;//服务二级菜单存储上一级菜单的进入位置以及地址
    struct linknode* last_pc1=&menu11;

    uint8 key = 0;
    while (1)
    {
        //更改进入菜单的条件时，下方条件同步更改
        if(!(gpio_get_level(DIP_IO_1) == 1 && gpio_get_level(DIP_IO_4) == 1)) break;//上电后若满足条件进入菜单，无此语句，想跳出菜单只能重新上电

        static int loact = 1;
        static int del_loact = 0;//用于删除上一级显示位置
        static int last_loact = 1;
        static int last_loact1 = 1;


        if (pc == &menu1 || pc == &menu2 || pc == &menu3 || pc == &menu4 )
        {
            //显示菜单项
            tft180_show_string(10, 00, "PID_con");
            tft180_show_string(10, 20, "menu2");
            tft180_show_string(10, 40, "menu3");
            tft180_show_string(10, 60, "menu4");
            tft180_show_string(0, (loact-1)*20, "*");

            //这一步是为了清除之前*所在位置的显示
            if(del_loact && del_loact != loact) tft180_show_string(0, (del_loact-1)*20, " ");



            key=Key_Scan();

            if (key == 1)
            {
                del_loact = loact;
                loact++;
                if (loact > 4) loact = 1;
                pc = pc->next;
            }
            else if (key == 2)
            {
                del_loact = loact;
                loact--;
                if (loact < 1) loact = 4;
                pc = (*pc).last;// pc->last;
            }
            else if (key == 3)
            {
                del_loact = loact;
                last_pc = pc;
                last_loact = loact;

                loact = 1;
                pc = pc->nextlev;

                pc->lastlev = last_pc;
            }
        }
        else if (pc == &menu11 || pc == &menu12 || pc == &menu13 || pc == &menu14)
        {
          
            tft180_show_string(10, 00, "menu11");
            tft180_show_string(10, 20, "menu12");
            tft180_show_string(10, 40, "menu13");
            tft180_show_string(10, 60, "menu14");
            tft180_show_string(0, (loact-1)*20, "*");

            if(del_loact && del_loact != loact) tft180_show_string(0, (del_loact-1)*20, " ");

            key=Key_Scan();
//
            if (key == 1)
            {
                del_loact = loact;
                loact++;
                if (loact > 4) loact = 1;
                pc = pc->next;
                pc->lastlev = last_pc;
            }
            else if (key == 2)
            {
                del_loact = loact;
                loact--;
                if (loact < 1) loact = 4;
                pc = pc->last;
                pc->lastlev = last_pc;
            }
            else if (key == 3)
            {
                del_loact = loact;
                last_pc1 = pc;
                last_loact1 = loact;
                loact = 1;
                pc = pc->nextlev;

                pc->lastlev = last_pc1;
            }
            else if (key == 4)
            {
                del_loact = loact;
                loact = last_loact;
                pc = pc->lastlev;
                tft180_clear();
            }
        }
        else if (pc == &menu111 || pc == &menu112 || pc == &menu113 || pc == &menu114)
        {
            tft180_show_string(10, 00, "menu111");
            tft180_show_string(10, 20, "menu112");
            tft180_show_string(10, 40, "menu113");
            tft180_show_string(10, 60, "menu114");
            tft180_show_string(0, (loact-1)*20, "*");

            if(del_loact && del_loact != loact) tft180_show_string(0, (del_loact-1)*20, " ");

            key=Key_Scan();

            if (key == 1)
            {
                del_loact = loact;
                loact++;
                if (loact > 4) loact = 1;
                pc = pc->next;
                pc->lastlev = last_pc1;
            }
            else if (key == 2)
            {
                del_loact = loact;
                loact--;
                if (loact < 1) loact = 4;
                pc = pc->last;
                pc->lastlev = last_pc1;
            }
            else if (key == 3)
            {
                if (loact == 1) pc->data += 10;
                else if (loact == 2) pc->data -= 10;

                printf("%d", pc->data);
                /*last_pc = pc;
                last_loact = loact;
                loact = 1;
                pc = pc->nextlev;

                pc->lastlev = last_pc;*/
            }
            else if (key == 4)
            {
                del_loact = loact;
                loact = last_loact1;
                pc = pc->lastlev;
                tft180_clear();
            }
        }
    }
}

#endif

#include "zf_common_headfile.h"

#define MENU_SHOW_MAX          (4)
#define MENU_TEXT_X            (10)
#define MENU_CURSOR_X          (0)
#define MENU_LINE_HEIGHT       (20)
#define MENU_ARRAY_COUNT(arr)  ((uint8)(sizeof(arr) / sizeof((arr)[0])))

typedef struct MenuPage MenuPage;
typedef void (*MenuAction)(int data);

typedef struct
{
    const char *name;
    int data;
    MenuPage *child;
    MenuAction action;
} MenuItem;

struct MenuPage
{
    const char *title;
    const MenuItem *items;
    uint8 count;
    MenuPage *parent;
};

uint8 keynum;
uint8 keynum1;

static MenuPage main_page;
static MenuPage pid_page;
static MenuPage angle_pid_page;
static MenuPage locat_pid_page;
static MenuPage high_pid_page;
static MenuPage image_page;
static MenuPage motor_page;
static MenuPage debug_page;

static void menu_default_action(int data)
{
    printf("select menu : %d\r\n", data);
}

static const MenuItem angle_pid_items[] =
{
    {"Pitch KP", 111, NULL, menu_default_action},
    {"Pitch KI", 112, NULL, menu_default_action},
    {"Pitch KD", 113, NULL, menu_default_action},
    {"Roll KP",  114, NULL, menu_default_action},
    {"Roll KI",  115, NULL, menu_default_action},
    {"Roll KD",  116, NULL, menu_default_action},
    {"Yaw KP",   117, NULL, menu_default_action},
    {"Yaw KI",   118, NULL, menu_default_action},
    {"Yaw KD",   119, NULL, menu_default_action},
};

static const MenuItem locat_pid_items[] =
{
    {"Locat X KP", 121, NULL, menu_default_action},
    {"Locat X KI", 122, NULL, menu_default_action},
    {"Locat X KD", 123, NULL, menu_default_action},
    {"Locat Y KP", 124, NULL, menu_default_action},
    {"Locat Y KI", 125, NULL, menu_default_action},
    {"Locat Y KD", 126, NULL, menu_default_action},
};

static const MenuItem high_pid_items[] =
{
    {"High Loca KP", 131, NULL, menu_default_action},
    {"High Speed KP", 132, NULL, menu_default_action},
    {"High Speed KD", 133, NULL, menu_default_action},
};

static const MenuItem pid_items[] =
{
    {"Angle PID", 11, &angle_pid_page, NULL},
    {"Locat PID", 12, &locat_pid_page, NULL},
    {"High PID",  13, &high_pid_page,  NULL},
};

static const MenuItem image_items[] =
{
    {"Image X", 21, NULL, menu_default_action},
    {"Image Y", 22, NULL, menu_default_action},
    {"Yaw Ctr", 23, NULL, menu_default_action},
    {"Y Target", 24, NULL, menu_default_action},
};

static const MenuItem motor_items[] =
{
    {"Base Spd", 31, NULL, menu_default_action},
    {"Limit Max", 32, NULL, menu_default_action},
    {"Limit Min", 33, NULL, menu_default_action},
};

static const MenuItem debug_items[] =
{
    {"TOF", 41, NULL, menu_default_action},
    {"PMW X", 42, NULL, menu_default_action},
    {"PMW Y", 43, NULL, menu_default_action},
    {"Yaw", 44, NULL, menu_default_action},
};

static const MenuItem main_items[] =
{
    {"PID_con", 1, &pid_page,   NULL},
    {"Image",   2, &image_page, NULL},
    {"Motor",   3, &motor_page, NULL},
    {"Debug",   4, &debug_page, NULL},
};

static void menu_page_set(MenuPage *page,
                          const char *title,
                          const MenuItem *items,
                          uint8 count,
                          MenuPage *parent)
{
    page->title = title;
    page->items = items;
    page->count = count;
    page->parent = parent;
}

static void menu_pages_init(void)
{
    menu_page_set(&main_page,      "Main",      main_items,      MENU_ARRAY_COUNT(main_items),      NULL);
    menu_page_set(&pid_page,       "PID",       pid_items,       MENU_ARRAY_COUNT(pid_items),       &main_page);
    menu_page_set(&angle_pid_page, "Angle PID", angle_pid_items, MENU_ARRAY_COUNT(angle_pid_items), &pid_page);
    menu_page_set(&locat_pid_page, "Locat PID", locat_pid_items, MENU_ARRAY_COUNT(locat_pid_items), &pid_page);
    menu_page_set(&high_pid_page,  "High PID",  high_pid_items,  MENU_ARRAY_COUNT(high_pid_items),  &pid_page);
    menu_page_set(&image_page,     "Image",     image_items,     MENU_ARRAY_COUNT(image_items),     &main_page);
    menu_page_set(&motor_page,     "Motor",     motor_items,     MENU_ARRAY_COUNT(motor_items),     &main_page);
    menu_page_set(&debug_page,     "Debug",     debug_items,     MENU_ARRAY_COUNT(debug_items),     &main_page);
}

static void menu_fix_top(const MenuPage *page, uint8 select, uint8 *top)
{
    if (select < *top)
    {
        *top = select;
    }
    else if (select >= (*top + MENU_SHOW_MAX))
    {
        *top = select - MENU_SHOW_MAX + 1;
    }

    if (page->count <= MENU_SHOW_MAX)
    {
        *top = 0;
    }
    else if (*top > (page->count - MENU_SHOW_MAX))
    {
        *top = page->count - MENU_SHOW_MAX;
    }
}

static void menu_show(const MenuPage *page, uint8 select, uint8 top)
{
    uint8 i = 0;
    uint8 item_index = 0;

    tft180_clear();

    for (i = 0; i < MENU_SHOW_MAX; i++)
    {
        item_index = top + i;

        if (item_index < page->count)
        {
            tft180_show_string(MENU_TEXT_X, i * MENU_LINE_HEIGHT, page->items[item_index].name);
            tft180_show_string(MENU_CURSOR_X, i * MENU_LINE_HEIGHT, (item_index == select) ? "*" : " ");
        }
        else
        {
            tft180_show_string(MENU_TEXT_X, i * MENU_LINE_HEIGHT, " ");
            tft180_show_string(MENU_CURSOR_X, i * MENU_LINE_HEIGHT, " ");
        }
    }
}

void DIP_init(void)
{
    gpio_init(DIP_IO_1, GPI, 1, GPI_PULL_UP);
    gpio_init(DIP_IO_2, GPI, 1, GPI_PULL_UP);
    gpio_init(DIP_IO_3, GPI, 1, GPI_PULL_UP);
    gpio_init(DIP_IO_4, GPI, 1, GPI_PULL_UP);
}

void Key_Init(void)
{
    key_init(10);
    gpio_init(key1, GPI, 1, GPI_PULL_UP);
    gpio_init(key2, GPI, 1, GPI_PULL_UP);
    gpio_init(key3, GPI, 1, GPI_PULL_UP);
    gpio_init(key4, GPI, 1, GPI_PULL_UP);
}

uint8 Key_Scan(void)
{
    key_scanner();
    if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
        return 1;
    else if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
        return 2;
    else if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
        return 3;
    else if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
        return 4;

    else if(key_get_state(KEY_1) == KEY_LONG_PRESS)
    {
        system_delay_ms(500);
        return 11;
    }
    else if(key_get_state(KEY_2) == KEY_LONG_PRESS)
    {
        system_delay_ms(500);
        return 22;
    }
    else if(key_get_state(KEY_3) == KEY_LONG_PRESS)
    {
        system_delay_ms(500);
        return 33;
    }
    else if(key_get_state(KEY_4) == KEY_LONG_PRESS)
    {
        system_delay_ms(500);
        return 44;
    }
    else
    {
        return 0;
    }
}

void sh_menu(void)
{
    MenuPage *current_page = &main_page;
    uint8 select = 0;
    uint8 top = 0;
    uint8 key = 0;

    menu_pages_init();
    menu_show(current_page, select, top);

    while (1)
    {
        if (!(gpio_get_level(DIP_IO_1) == 1 && gpio_get_level(DIP_IO_4) == 1))
        {
            tft180_clear();
            break;
        }

        key = Key_Scan();

        if (key == 1)
        {
            select++;
            if (select >= current_page->count)
            {
                select = 0;
            }

            menu_fix_top(current_page, select, &top);
            menu_show(current_page, select, top);
        }
        else if (key == 2)
        {
            if (select == 0)
            {
                select = current_page->count - 1;
            }
            else
            {
                select--;
            }

            menu_fix_top(current_page, select, &top);
            menu_show(current_page, select, top);
        }
        else if (key == 3)
        {
            const MenuItem *item = &current_page->items[select];

            if (item->child != NULL)
            {
                current_page = item->child;
                select = 0;
                top = 0;
                menu_show(current_page, select, top);
            }
            else if (item->action != NULL)
            {
                item->action(item->data);
                menu_show(current_page, select, top);
            }
            else
            {
                menu_default_action(item->data);
                menu_show(current_page, select, top);
            }
        }
        else if (key == 4)
        {
            if (current_page->parent != NULL)
            {
                current_page = current_page->parent;
                select = 0;
                top = 0;
                menu_show(current_page, select, top);
            }
        }
    }
}

