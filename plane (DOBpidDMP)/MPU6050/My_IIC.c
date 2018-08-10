#include "include.h"
#include "My_IIC.h"

unsigned char ack;

void DelayUs(unsigned int us)
{
  int ii,jj;
  if (us<1) us=1;
  for(ii=0;ii<us;ii++)
    for(jj=0;jj<1;jj++);   //50MHz--1us
}

void DelayMs(unsigned int ms)
{
  int ii,jj;
  if (ms<1) ms=1;
  for(ii=0;ii<ms;ii++)
    for(jj=0;jj<8800;jj++);   //50MHz--1ms
}

void get_ms(unsigned long *time)
{
    
}
/**
* @fn:         IIC_Init
* @bref:       IIC port initialization
* @params:     none    
* @return:     none
* @version:    0.1
* @date:       2013-1-5 
* 
*/
void IIC_Init(void)
{
  I2C_DAT_OUT();
  I2C_CLK_INIT();
}
/**
* @fn:         IIC_Start
* @bref:       generate IIC start signal
* @params:     none    
* @return:     none
* @version:    0.1
* @date:       2013-1-5 
* 
*/
void IIC_Start(void)
{
  SDA_OUT();
  IIC_SDA(1);
  //IIC_DelayUs(1);
  IIC_SCL(1);
  IIC_DelayUs(1);
  IIC_SDA(0);
  IIC_DelayUs(1);
  IIC_SCL(0);
  //IIC_DelayUs(1);
}
/**
* @fn:         IIC_SendACK
* @bref:       generate IIC stop signal
* @params:     none    
* @return:     none
* @version:    0.1
* @date:       2013-1-5 
* 
*/

/**
* @fn:         IIC_SendByte
* @bref:       IIC sned one byte
* @params:     byte to send    
* @return:     none
* @version:    0.1
* @date:       2013-1-5 
* 
*/

void IIC_SendByte(unsigned char dat)
{
  unsigned char BitCnt;

  SDA_OUT();  
  for(BitCnt=0;BitCnt<8;BitCnt++)   /*Ҫ���͵����ݳ���Ϊ8λ*/
  {
    if((dat<<BitCnt)&0x80) IIC_SDA(1);   /*�жϷ���λ*/
      else IIC_SDA(0);                
    IIC_DelayUs(1);
    IIC_SCL(1);              /*��ʱ����Ϊ�ߣ�֪ͨ��������ʼ��������λ*/
    
    IIC_DelayUs(1);              /*��֤ʱ�Ӹߵ�ƽ���ڴ���4��s*/
           
    IIC_SCL(0); 
  }
  
  IIC_DelayUs(1);
  SDA_IN();                 /*8λ��������ͷ������ߣ�׼������Ӧ��λ*/
  IIC_DelayUs(1);  
  IIC_SCL(1);
  IIC_DelayUs(1);
  if(READ_SDA())ack=0;     
    else ack=1;         /*�ж��Ƿ���յ�Ӧ���ź�*/
  IIC_SCL(0);
  IIC_DelayUs(1); 
}

void IIC_Stop(void)
{
  SDA_OUT();
  IIC_SDA(0);
  IIC_DelayUs(1);
  IIC_SCL(1);
  IIC_DelayUs(1);
  IIC_SDA(1);
  //IIC_DelayUs(1);							   	
}


void IIC_SendACK(unsigned char a)  
{  
  SDA_OUT();  
  if(a==0) IIC_SDA(0);           /*�ڴ˷���Ӧ����Ӧ���ź� */
       else IIC_SDA(1);
  IIC_DelayUs(1);      
  IIC_SCL(1);
  
  IIC_DelayUs(1);                     /*ʱ�ӵ͵�ƽ���ڴ���4��s*/
  
  IIC_SCL(0);                      /*��ʱ���ߣ�ǯסI2C�����Ա��������*/
  IIC_DelayUs(1);    
}  
/**
* @fn:         IIC_ReadByte
* @bref:       IIC read one byte
* @params:     byte to read    
* @return:     none
* @version:    0.1
* @date:       2013-1-5 
* 
*/
unsigned char IIC_ReadByte(void)
{
   unsigned char retc;
   unsigned char BitCnt;
  
   retc=0; 
   SDA_IN();               /*��������Ϊ���뷽ʽ*/
   for(BitCnt=0;BitCnt<8;BitCnt++)
   {
     IIC_DelayUs(1);   
     IIC_SCL(0);                  /*��ʱ����Ϊ�ͣ�׼����������λ*/
    
     IIC_DelayUs(1);                  /*ʱ�ӵ͵�ƽ���ڴ���4.7��s*/
   
     IIC_SCL(1);                  /*��ʱ����Ϊ��ʹ��������������Ч*/
     IIC_DelayUs(1);
     retc=retc<<1;
     if(READ_SDA()) retc=retc+1;   /*������λ,���յ�����λ����retc�� */
     IIC_DelayUs(1); 
   }
   IIC_SCL(0);   
   IIC_DelayUs(1);
   return(retc);
}

char i2cWriteBuffer(unsigned char addr, unsigned char reg, unsigned char len, unsigned char * data)
{  
    unsigned char i;

    IIC_Start();                /*��������*/
    IIC_SendByte(addr<<1 + 0);              /*����������ַ*/
    if(ack==0)return(1);
    IIC_SendByte(reg);             /*���������ӵ�ַ*/
    if(ack==0)return(1);
    for(i=0;i<len;i++)
    {   
      IIC_SendByte(*data);             /*��������*/
      if(ack==0)return(1);
      data++;
    } 
    IIC_Stop();                 /*��������*/ 

    return(0);
}  

char i2cRead(unsigned char addr, unsigned char reg, unsigned char len, unsigned char *buf)
{  
    unsigned char i;

    IIC_Start();                 /*��������*/
    IIC_SendByte((addr<<1) + 0);                 /*����������ַ*/
    if(ack==0)return(1);
    IIC_SendByte(reg);                /*���������ӵ�ַ*/
    if(ack==0)return(1);
    IIC_Start();     /*������������*/
    IIC_SendByte((addr<<1) + 1);
    if(ack==0)return(1);
    for(i=0;i<len-1;i++)
    {   
      *buf=IIC_ReadByte();                /*��������*/
      IIC_SendACK(0);                 /*���;ʹ�λ*/  
      buf++;
    } 
    *buf=IIC_ReadByte();
    IIC_SendACK(1);                    /*���ͷ�Ӧλ*/
    IIC_Stop();                     /*��������*/ 
    return(0);
}  

unsigned char HMC5883_RecvByte(void)
{
  unsigned char i,receive=0;
  SDA_IN();
  for(i=0;i<8;i++ )
  {
    IIC_SCL(0); 
    DelayUs(1);
    IIC_SCL(1);
    receive<<=1;
    if(READ_SDA())
      receive++;   
    DelayUs(1); 
  }
  return receive;
}
void HMC5883_SendByte(unsigned char dat)
{
  unsigned char i;
  SDA_OUT();
  for(i=0;i<8;i++)
  {
    if(dat&0x80) 
      IIC_SDA(1);
    else 
      IIC_SDA(0);
    dat<<=1;
    DelayUs(1);
    IIC_SCL(1);
    DelayUs(1);
    IIC_SCL(0);
  }
  IIC_RecvACK();
}







