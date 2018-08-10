#include "common.h"
#include "include.h"






GPIO_InitTypeDef pta_init;
GPIO_InitTypeDef ptb_init;
GPIO_InitTypeDef ptc_init;
GPIO_InitTypeDef ptd_init;
GPIO_InitTypeDef pte_init;
DMA_InitTypeDef dma_init_struct;

 
uint32 V_Cnt=0,H_Cnt=0;                     //行采集计数
uint8 Is_DispPhoto=0;               //图像发送标志

void gpio_init()
{ 
  //OV数据口初始化：PTe0~PTe7
  pte_init.GPIO_PTx = PTE;
  pte_init.GPIO_Dir = DIR_INPUT;
  pte_init.GPIO_Pins = GPIO_Pin0_7;
  pte_init.GPIO_PinControl = IRQC_DIS | INPUT_PULL_DIS;
  LPLD_GPIO_Init(pte_init);
  
  //OV行信号接口初始化：PTD13-H
  ptd_init.GPIO_PTx = PTD;
  ptd_init.GPIO_Dir = DIR_INPUT;
  ptd_init.GPIO_Pins = GPIO_Pin13;
  ptd_init.GPIO_PinControl = IRQC_RI|INPUT_PULL_DOWN;
  ptd_init.GPIO_Isr = portd_isr;
  LPLD_GPIO_Init(ptd_init); 
  
  //OV场信号接口初始化：PTB15-V 奇数场
  ptd_init.GPIO_PTx = PTD;
  ptd_init.GPIO_Dir = DIR_INPUT;
  ptd_init.GPIO_Pins = GPIO_Pin15;
  ptd_init.GPIO_PinControl = IRQC_FA | INPUT_PULL_UP;
  ptd_init.GPIO_Isr = portd_isr;
  LPLD_GPIO_Init(ptd_init); 
  
  //OV DMA触发信号接口初始化：PTC1-PCLK&HREF
  ptd_init.GPIO_PTx = PTD;
  ptd_init.GPIO_Pins = GPIO_Pin12;
  ptd_init.GPIO_Dir = DIR_INPUT;
  ptd_init.GPIO_PinControl = IRQC_DMARI | INPUT_PULL_DIS;//上升沿触发
  LPLD_GPIO_Init(ptd_init); 
  
}

void dma_init()
{
  //DMA参数配置
  dma_init_struct.DMA_CHx = DMA_CH0;    //CH0通道
  dma_init_struct.DMA_Req = PORTD_DMAREQ;       //PORTE为请求源
  
  dma_init_struct.DMA_MajorLoopCnt = H; //主循环计数值：行采集点数，宽度
  dma_init_struct.DMA_MinorByteCnt = 1; //次循环字节计数：每次读入1字节
  dma_init_struct.DMA_SourceAddr = (uint32)&PTE->PDIR;        //源地址：PTE0~7
  dma_init_struct.DMA_DestAddr = (uint32 )BUFF;      //目的地址：存放图像的数组
  dma_init_struct.DMA_SourceDataSize = DMA_SRC_8BIT;
  dma_init_struct.DMA_DestDataSize = DMA_DST_8BIT;
  dma_init_struct.DMA_DestAddrOffset = 1;       //目的地址偏移：每次读入增加1
  dma_init_struct.DMA_AutoDisableReq = TRUE;    //自动禁用请求
  //初始化DMA
  LPLD_DMA_Init(dma_init_struct);
}

uint8 line_complete = 0;
void portd_isr()
{
  if(LPLD_GPIO_IsPinxExt(PORTD, GPIO_Pin15) ) //奇数场
  {
    LPLD_GPIO_ClearIntFlag(PORTD);
    //检测到场开始信号，加载目的地址
   // H_Cnt=1;
   
    LPLD_DMA_LoadDstAddr(DMA_CH0, (BUFF));
    //清行中断标志，防止进入无效行中断
    //enable_irq(PORTD_IRQn);
    line_complete = 0;
  }
  
  
  if( LPLD_GPIO_IsPinxExt(PORTD, GPIO_Pin13) && !line_complete)   //行
  {
    LPLD_GPIO_ClearIntFlag(PORTD);
  // if(H_Cnt==1)
  //检测到行开始信号，使能DMA请求
    if(V_Cnt<V)
    {
      LPLD_DMA_EnableReq(DMA_CH0);  
      V_Cnt++;                   
    }
    //行数采集已满，关闭中断
    else
    {
      //关GPIO中断 
      //disable_irq(PORTA_IRQn);
     // disable_irq(PORTB_IRQn);
      Is_DispPhoto = 1;//可以显示图像
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


