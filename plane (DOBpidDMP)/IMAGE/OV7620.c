#include "common.h"
#include "include.h"






GPIO_InitTypeDef pta_init;
GPIO_InitTypeDef ptb_init;
GPIO_InitTypeDef ptc_init;
GPIO_InitTypeDef ptd_init;
GPIO_InitTypeDef pte_init;
DMA_InitTypeDef dma_init_struct;

 
uint32 V_Cnt=0,H_Cnt=0;                     //�вɼ�����
uint8 Is_DispPhoto=0;               //ͼ���ͱ�־

void gpio_init()
{ 
  //OV���ݿڳ�ʼ����PTe0~PTe7
  pte_init.GPIO_PTx = PTE;
  pte_init.GPIO_Dir = DIR_INPUT;
  pte_init.GPIO_Pins = GPIO_Pin0_7;
  pte_init.GPIO_PinControl = IRQC_DIS | INPUT_PULL_DIS;
  LPLD_GPIO_Init(pte_init);
  
  //OV���źŽӿڳ�ʼ����PTD13-H
  ptd_init.GPIO_PTx = PTD;
  ptd_init.GPIO_Dir = DIR_INPUT;
  ptd_init.GPIO_Pins = GPIO_Pin13;
  ptd_init.GPIO_PinControl = IRQC_RI|INPUT_PULL_DOWN;
  ptd_init.GPIO_Isr = portd_isr;
  LPLD_GPIO_Init(ptd_init); 
  
  //OV���źŽӿڳ�ʼ����PTB15-V ������
  ptd_init.GPIO_PTx = PTD;
  ptd_init.GPIO_Dir = DIR_INPUT;
  ptd_init.GPIO_Pins = GPIO_Pin15;
  ptd_init.GPIO_PinControl = IRQC_FA | INPUT_PULL_UP;
  ptd_init.GPIO_Isr = portd_isr;
  LPLD_GPIO_Init(ptd_init); 
  
  //OV DMA�����źŽӿڳ�ʼ����PTC1-PCLK&HREF
  ptd_init.GPIO_PTx = PTD;
  ptd_init.GPIO_Pins = GPIO_Pin12;
  ptd_init.GPIO_Dir = DIR_INPUT;
  ptd_init.GPIO_PinControl = IRQC_DMARI | INPUT_PULL_DIS;//�����ش���
  LPLD_GPIO_Init(ptd_init); 
  
}

void dma_init()
{
  //DMA��������
  dma_init_struct.DMA_CHx = DMA_CH0;    //CH0ͨ��
  dma_init_struct.DMA_Req = PORTD_DMAREQ;       //PORTEΪ����Դ
  
  dma_init_struct.DMA_MajorLoopCnt = H; //��ѭ������ֵ���вɼ����������
  dma_init_struct.DMA_MinorByteCnt = 1; //��ѭ���ֽڼ�����ÿ�ζ���1�ֽ�
  dma_init_struct.DMA_SourceAddr = (uint32)&PTE->PDIR;        //Դ��ַ��PTE0~7
  dma_init_struct.DMA_DestAddr = (uint32 )BUFF;      //Ŀ�ĵ�ַ�����ͼ�������
  dma_init_struct.DMA_SourceDataSize = DMA_SRC_8BIT;
  dma_init_struct.DMA_DestDataSize = DMA_DST_8BIT;
  dma_init_struct.DMA_DestAddrOffset = 1;       //Ŀ�ĵ�ַƫ�ƣ�ÿ�ζ�������1
  dma_init_struct.DMA_AutoDisableReq = TRUE;    //�Զ���������
  //��ʼ��DMA
  LPLD_DMA_Init(dma_init_struct);
}

uint8 line_complete = 0;
void portd_isr()
{
  if(LPLD_GPIO_IsPinxExt(PORTD, GPIO_Pin15) ) //������
  {
    LPLD_GPIO_ClearIntFlag(PORTD);
    //��⵽����ʼ�źţ�����Ŀ�ĵ�ַ
   // H_Cnt=1;
   
    LPLD_DMA_LoadDstAddr(DMA_CH0, (BUFF));
    //�����жϱ�־����ֹ������Ч���ж�
    //enable_irq(PORTD_IRQn);
    line_complete = 0;
  }
  
  
  if( LPLD_GPIO_IsPinxExt(PORTD, GPIO_Pin13) && !line_complete)   //��
  {
    LPLD_GPIO_ClearIntFlag(PORTD);
  // if(H_Cnt==1)
  //��⵽�п�ʼ�źţ�ʹ��DMA����
    if(V_Cnt<V)
    {
      LPLD_DMA_EnableReq(DMA_CH0);  
      V_Cnt++;                   
    }
    //�����ɼ��������ر��ж�
    else
    {
      //��GPIO�ж� 
      //disable_irq(PORTA_IRQn);
     // disable_irq(PORTB_IRQn);
      Is_DispPhoto = 1;//������ʾͼ��
      V_Cnt=0;
     // H_Cnt=0; 
      //disable_irq(PORTD_IRQn);
      line_complete = 1;
      
      sendCamImgToCamViewer();
      //find_blackblock_h();
      //find_spot();
      //send_data();
    }
  
    
    //LPLD_DMA_DisableReq(DMA_CH0);
  }
}


