#ifndef _SAFTY_H_
#define _SAFTY_H_

#define _BZ_ON   LPLD_GPIO_Output_b(PTE, 12, 1)
#define _BZ_OFF  LPLD_GPIO_Output_b(PTE, 12, 0)


extern uint8 flag_dump;//�̵�����������
extern uint8 flag_lock;//�ɿ���������

void relay_init();
void MCU_Reset();//���Ź���λ������
void safty_lock();//���ڷɿؽ���
void BEEP_init();
void check_unbalance();
void quick_land();
#endif