#ifndef _SAFTY_H_
#define _SAFTY_H_

#define _BZ_ON   LPLD_GPIO_Output_b(PTE, 12, 1)
#define _BZ_OFF  LPLD_GPIO_Output_b(PTE, 12, 0)


extern uint8 flag_dump;//继电器掉电命令
extern uint8 flag_lock;//飞控锁定命令

void relay_init();
void MCU_Reset();//看门狗复位处理器
void safty_lock();//用于飞控解锁
void BEEP_init();
void check_unbalance();
void quick_land();
#endif