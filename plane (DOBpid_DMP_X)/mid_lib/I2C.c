//I2C 通用C文件 IO模拟版
//DDK
//2015 5 4

#include "common.h"
#include "include.h"

void I2C_CLK_INIT()
{
  GPIO_InitTypeDef gpio_inittype;
  gpio_inittype.GPIO_PTx = I2C_CLK_PT;
  gpio_inittype.GPIO_Pins = I2C_CLK_PIN;
  gpio_inittype.GPIO_Dir = DIR_OUTPUT;
  gpio_inittype.GPIO_Output = OUTPUT_H;
  gpio_inittype.GPIO_PinControl = IRQC_DIS;
  LPLD_GPIO_Init(gpio_inittype);
}

void I2C_DAT_IN()
{
  GPIO_InitTypeDef gpio_inittype;
  gpio_inittype.GPIO_PTx = I2C_DAT_PT;
  gpio_inittype.GPIO_Pins = I2C_DAT_PIN;
  gpio_inittype.GPIO_Dir = DIR_INPUT;
  gpio_inittype.GPIO_Output = OUTPUT_H;
  gpio_inittype.GPIO_PinControl = IRQC_DIS;
  LPLD_GPIO_Init(gpio_inittype);
}

void I2C_DAT_OUT()
{
  GPIO_InitTypeDef gpio_inittype;
  gpio_inittype.GPIO_PTx = I2C_DAT_PT;
  gpio_inittype.GPIO_Pins = I2C_DAT_PIN;
  gpio_inittype.GPIO_Dir = DIR_INPUT;
  gpio_inittype.GPIO_Output = OUTPUT_H;
  gpio_inittype.GPIO_PinControl = IRQC_DIS;
  LPLD_GPIO_Init(gpio_inittype);
}

//**************************************
//I2C起始信号
//**************************************
//--------------------------------------------------------------------------------------------------
// 函数名称： iic_start()
// 函数功能： 启动I2C总线子程序
//--------------------------------------------------------------------------------------------------
void I2C_start(void)
{    
  I2CDDRA = 1;
  I2CSDA = 1;  
  I2Cdelay_1us(1);
  I2CCLK = 1;
  I2Cdelay_1us(1);      // 延时5us 
  I2CSDA = 0;
  I2Cdelay_1us(1);  
  I2CCLK = 0;
  I2Cdelay_1us(1);
}


//**************************************
//I2C停止信号
//**************************************
//--------------------------------------------------------------------------------------------------
// 函数名称： iic_stop()
// 函数功能： 停止I2C总线数据传送子程序
//--------------------------------------------------------------------------------------------------
void I2C_stop(void)
{ 
  I2CDDRA = 1;
  I2CSDA = 0;   	   //时钟保持高，数据线从低到高一次跳变，I2C通信停止
  I2Cdelay_1us(1);      // 延时1us 
  I2CCLK = 1;
  I2Cdelay_1us(1);
  I2CSDA = 1;
  I2Cdelay_1us(1);
  //BFCLK = 0;
  //delay(2);
}
//--------------------------------------------------------------------------------------------------
// 函数名称： check_ACK
// 函数功能： 主机应答位检查子程序，迫使数据传输过程结束
//--------------------------------------------------------------------------------------------------
uint8 I2Ccheck_ACK(void)
{ 
  uint8 check ;

  I2CCLK = 1;
 I2Cdelay_1us(1);
  check = 0;
  I2CDDRA = 0;
  if(I2CSDAI == 1)    // 若BFSDA=1 表明非应答，置位非应答标志F0
  check = 1;
  I2Cdelay_1us(1);      // 延时1us 
  I2CCLK = 0;
  //gpio_init (PORTE , 12, GPO,HIGH);      
  I2CDDRA = 1 ;         //
  //BFSDA = 1 ;
  return  check ;
}
//**************************************
//I2C接收应答信号
//**************************************
//--------------------------------------------------------------------------------------------------
// 函数名称： slave_ACK
// 函数功能： 从机发送应答位子程序
//--------------------------------------------------------------------------------------------------
void I2Cslave_ACK(void)
{
  I2CDDRA = 1;
  I2CSDA = 1; 
  I2Cdelay_1us(1);      // 延时1us 
  I2CCLK = 1;
  I2Cdelay_1us(1);			
  
  I2CCLK = 0;
  I2Cdelay_1us(1);
}

//--------------------------------------------------------------------------------------------------
// 函数名称： slave_NOACK
// 函数功能： 从机发送非应答位子程序，迫使数据传输过程结束
//--------------------------------------------------------------------------------------------------
void I2Cslave_NOACK(void)
{ 
  I2CDDRA = 1;
  I2CSDA = 0;  
  I2Cdelay_1us(1);      // 延时1us 
  I2CCLK = 1;
  I2Cdelay_1us(1);
  //BFSDA = 0;
  //delay(5);      // 延时1us 
  I2CCLK = 0;
}
//--------------------------------------------------------------------------------------------------
// 函数名称： IICSendByte
// 入口参数： ch
// 函数功能： 发送一个字节
//--------------------------------------------------------------------------------------------------
void I2CSendByte(uint8 ch)
{ 
  uint8_t i;     // 向BFSDA上发送一位数据字节，共八位
  for (i=0; i<8; i++)         //8位计数器
  {
                  //移出数据的最高位
    I2CDDRA = 1;
    I2CSDA = ((ch&0x80) >> 7);               //送数据口
    I2CCLK = 1;                //拉高时钟线
    I2Cdelay_1us(1);             //延时
    I2CCLK = 0;                //拉低时钟线
    I2Cdelay_1us(1);             //延时
    ch <<= 1;
  }
  I2Ccheck_ACK();
}
//--------------------------------------------------------------------------------------------------
// 函数名称： IICreceiveByte
// 返回接收的数据
// 函数功能： 接收一字节子程序
//--------------------------------------------------------------------------------------------------
uint8 I2CreceiveByte(void)
{
  uint8 n=8;    // 从BFSDA线上读取一上数据字节，共八位
  uint8 tdata = 0;
  while(n--)
  {      
    I2CDDRA = 1 ;
    I2CSDA = 1;
    I2Cdelay_1us(1);
    I2CCLK=0;
    I2Cdelay_1us(1);
    I2CCLK = 1;
    I2Cdelay_1us(1);
    tdata = tdata<<1;    // 左移一位，或_crol_(temp,1)
    if(I2CSDAI == 1)
    tdata = tdata|0x01;    // 若接收到的位为1，则数据的最后一位置1
    else 
    tdata = tdata&0xfe;    // 否则数据的最后一位置0
    I2CCLK=0;
    I2CDDRA = 1 ;
  }
  return(tdata);
}



