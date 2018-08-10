//I2C 通用H文件 IO模拟版
//DDK
//2015 5 4

#ifndef _I2C_H_
#define _I2C_H_

#define I2C_CLK_PT PTD
#define I2C_DAT_PT PTD
#define I2C_CLK_PIN GPIO_Pin8
#define I2C_DAT_PIN GPIO_Pin9

#define I2C_CLK_NUM 8
#define I2C_DAT_NUM 9


#define I2CSDA      PTD9_O                       
#define I2CCLK      PTD8_O                      
#define I2CSDAI     PTD9_I                             
#define I2CDDRA     DDRD9                        


//#define IICEorr    (0)
//#define IICOK      (1)

void I2C_CLK_INIT();
void I2C_DAT_OUT();

void I2C_start(void) ;
void I2C_stop(void) ;
void I2Cslave_ACK(void) ;
void I2Cslave_NOACK(void) ;
uint8 I2Ccheck_ACK(void);
void I2CSendByte(uint8 ch) ;
uint8 I2CreceiveByte(void) ;

//****************************************

#endif