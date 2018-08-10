//I2C ͨ��C�ļ� IOģ���
//DDK
//2015 5 4

#include "common.h"
#include "include.h"

void I2Cdelay_1us(uint8_t us)                 // 1/15 us��ʱ����
{
  volatile uint8_t i ,j ;
  if(us < 1 )  us = 1 ;
  for(i=0;i<us;i++) 
  {
    for(j = 0 ;j < 1 ;j ++)
      ;    
  }  
}

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
//I2C��ʼ�ź�
//**************************************
//--------------------------------------------------------------------------------------------------
// �������ƣ� iic_start()
// �������ܣ� ����I2C�����ӳ���
//--------------------------------------------------------------------------------------------------
void I2C_start(void)
{    
  I2CDDRA = 1;
  I2CSDA = 1;  
  I2Cdelay_1us(1);
  I2CCLK = 1;
  I2Cdelay_1us(1);      // ��ʱ5us 
  I2CSDA = 0;
  I2Cdelay_1us(1);  
  I2CCLK = 0;
  I2Cdelay_1us(1);
}


//**************************************
//I2Cֹͣ�ź�
//**************************************
//--------------------------------------------------------------------------------------------------
// �������ƣ� iic_stop()
// �������ܣ� ֹͣI2C�������ݴ����ӳ���
//--------------------------------------------------------------------------------------------------
void I2C_stop(void)
{ 
  I2CDDRA = 1;
  I2CSDA = 0;   	   //ʱ�ӱ��ָߣ������ߴӵ͵���һ�����䣬I2Cͨ��ֹͣ
  I2Cdelay_1us(1);      // ��ʱ1us 
  I2CCLK = 1;
  I2Cdelay_1us(1);
  I2CSDA = 1;
  I2Cdelay_1us(1);
  //BFCLK = 0;
  //delay(2);
}
//--------------------------------------------------------------------------------------------------
// �������ƣ� check_ACK
// �������ܣ� ����Ӧ��λ����ӳ�����ʹ���ݴ�����̽���
//--------------------------------------------------------------------------------------------------
uint8 I2Ccheck_ACK(void)
{ 
  uint8 check ;

  I2CCLK = 1;
 I2Cdelay_1us(1);
  check = 0;
  I2CDDRA = 0;
  if(I2CSDAI == 1)    // ��BFSDA=1 ������Ӧ����λ��Ӧ���־F0
  check = 1;
  I2Cdelay_1us(1);      // ��ʱ1us 
  I2CCLK = 0;
  //gpio_init (PORTE , 12, GPO,HIGH);      
  I2CDDRA = 1 ;         //
  //BFSDA = 1 ;
  return  check ;
}
//**************************************
//I2C����Ӧ���ź�
//**************************************
//--------------------------------------------------------------------------------------------------
// �������ƣ� slave_ACK
// �������ܣ� �ӻ�����Ӧ��λ�ӳ���
//--------------------------------------------------------------------------------------------------
void I2Cslave_ACK(void)
{
  I2CDDRA = 1;
  I2CSDA = 1; 
  I2Cdelay_1us(1);      // ��ʱ1us 
  I2CCLK = 1;
  I2Cdelay_1us(1);			
  
  I2CCLK = 0;
  I2Cdelay_1us(1);
}

//--------------------------------------------------------------------------------------------------
// �������ƣ� slave_NOACK
// �������ܣ� �ӻ����ͷ�Ӧ��λ�ӳ�����ʹ���ݴ�����̽���
//--------------------------------------------------------------------------------------------------
void I2Cslave_NOACK(void)
{ 
  I2CDDRA = 1;
  I2CSDA = 0;  
  I2Cdelay_1us(1);      // ��ʱ1us 
  I2CCLK = 1;
  I2Cdelay_1us(1);
  //BFSDA = 0;
  //delay(5);      // ��ʱ1us 
  I2CCLK = 0;
}
//--------------------------------------------------------------------------------------------------
// �������ƣ� IICSendByte
// ��ڲ����� ch
// �������ܣ� ����һ���ֽ�
//--------------------------------------------------------------------------------------------------
void I2CSendByte(uint8 ch)
{ 
  uint8_t i;     // ��BFSDA�Ϸ���һλ�����ֽڣ�����λ
  for (i=0; i<8; i++)         //8λ������
  {
                  //�Ƴ����ݵ����λ
    I2CDDRA = 1;
    I2CSDA = ((ch&0x80) >> 7);               //�����ݿ�
    I2CCLK = 1;                //����ʱ����
    I2Cdelay_1us(1);             //��ʱ
    I2CCLK = 0;                //����ʱ����
    I2Cdelay_1us(1);             //��ʱ
    ch <<= 1;
  }
  I2Ccheck_ACK();
}
//--------------------------------------------------------------------------------------------------
// �������ƣ� IICreceiveByte
// ���ؽ��յ�����
// �������ܣ� ����һ�ֽ��ӳ���
//--------------------------------------------------------------------------------------------------
uint8 I2CreceiveByte(void)
{
  uint8 n=8;    // ��BFSDA���϶�ȡһ�������ֽڣ�����λ
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
    tdata = tdata<<1;    // ����һλ����_crol_(temp,1)
    if(I2CSDAI == 1)
    tdata = tdata|0x01;    // �����յ���λΪ1�������ݵ����һλ��1
    else 
    tdata = tdata&0xfe;    // �������ݵ����һλ��0
    I2CCLK=0;
    I2CDDRA = 1 ;
  }
  return(tdata);
}




