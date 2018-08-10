#ifndef _OV7620_H_
#define _OV7620_H_

#define H 140
#define V 300
#define PHOTO_SIZE H*V

void portd_isr();
void dma_init();
void gpio_init();
extern uint8 BUFF[PHOTO_SIZE];


#endif 
