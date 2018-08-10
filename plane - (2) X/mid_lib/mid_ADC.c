/*******
*******ADC中层库，链接拉普兰德库，使其更易使用
******
******/
#include "common.h"
#include "include.h"




void ADC_Init_solf ( ADC_Type *Adcx , uint8 BitMode  )
{
  ADC_InitTypeDef adc_init_struct;
  adc_init_struct.ADC_Adcx = Adcx;             /**ADC号*/
  adc_init_struct.ADC_BitMode = BitMode;      /*精度*/
  adc_init_struct.ADC_CalEnable = TRUE;        /*最好初始化为真*///使能初始化校验
  adc_init_struct.ADC_HwAvgSel = HW_4AVG;      //使能4次硬件校准
  adc_init_struct.ADC_SampleTimeCfg = SAMTIME_SHORT;    //短采样时间
  adc_init_struct.ADC_CalEnable = TRUE; //使能初始化校验
  adc_init_struct.ADC_DmaEnable = FALSE;        //不使能DMA请求
  adc_init_struct.ADC_DiffMode=ADC_SE;
  adc_init_struct.ADC_LongSampleTimeSel = LSAMTIME_20EX;
  adc_init_struct.ADC_PgaGain =  PGA_1GAIN;
  adc_init_struct.ADC_MuxSel=MUX_ADXXA;
    
    
  LPLD_ADC_Init(adc_init_struct);      
}

/*
void ADC_Init_hard ( ADC_Type *Adcx ，uint8  Req，uint32 *aim_Addr)//强制12位采集方便DMA
{
   **********************************************
  //Step 1.配置ADC0和ADC1的参数
  //ADC0-A
  adc0_init_struct.ADC_Adcx = ADC0;
  adc0_init_struct.ADC_BitMode = SE_12BIT;      //12位精度
  adc0_init_struct.ADC_CalEnable = TRUE;        //使能初始化校验
  adc0_init_struct.ADC_HwAvgSel = HW_4AVG;      //使能4次硬件校准
  adc0_init_struct.ADC_HwTrgCfg = HW_TRGA;      //配置A组为硬件触发
  adc0_init_struct.ADC_DmaEnable = TRUE;        //使能DMA请求
  LPLD_ADC_Init(adc0_init_struct);      //初始化ADC0-A组通道
  //ADC0-B，只需配置不同参数的成员变量
  adc0_init_struct.ADC_HwTrgCfg = HW_TRGB;      //配置B组为硬件触发
  LPLD_ADC_Init(adc0_init_struct);      //初始化ADC0-B组通道
  //使能ADC0的通道DAD1、AD14的引脚复用功能
  LPLD_ADC_Chn_Enable(ADC0, DAD1);
  LPLD_ADC_Chn_Enable(ADC0, AD14);
  //使能ADC0的相关转换通道
  LPLD_ADC_EnableConversion(ADC0, DAD1, 0, FALSE);
  LPLD_ADC_EnableConversion(ADC0, AD14, 1, FALSE);
  
  //ADC1A
  adc1_init_struct.ADC_Adcx = ADC1;
  adc1_init_struct.ADC_BitMode = SE_12BIT;
  adc1_init_struct.ADC_CalEnable = TRUE;
  adc1_init_struct.ADC_HwAvgSel = HW_4AVG;
  adc1_init_struct.ADC_HwTrgCfg = HW_TRGA;
  adc1_init_struct.ADC_DmaEnable = TRUE;
  LPLD_ADC_Init(adc1_init_struct);
  //ADC1B
  adc1_init_struct.ADC_HwTrgCfg = HW_TRGB;
  LPLD_ADC_Init(adc1_init_struct);
  LPLD_ADC_Chn_Enable(ADC1, AD10);
  LPLD_ADC_Chn_Enable(ADC1, AD11);
  LPLD_ADC_EnableConversion(ADC1, AD10, 0, FALSE);
  LPLD_ADC_EnableConversion(ADC1, AD11, 1, FALSE);
    
  **********************************************
  //Step 2.配置DMA CH0和DMA CH1，分别处理ADC0和ADC1的DMA请求
  //DMA CH0
  dma0_init_struct.DMA_CHx = DMA_CH0;           //使用Ch0通道
  dma0_init_struct.DMA_Req = ADC0_DMAREQ;       //DMA请求源为ADC0
  dma0_init_struct.DMA_MajorLoopCnt = 2;        //主循环计数2次,因为要循环ADC0的AB两组通道
  dma0_init_struct.DMA_MinorByteCnt = 2;        //次循环传输字节计数（由于ADC采样为12位，因此传输2字节）
  dma0_init_struct.DMA_SourceAddr = (uint32)&(ADC0->R[0]);       //源地址：ADC0结果寄存器A地址
  dma0_init_struct.DMA_SourceDataSize = DMA_SRC_16BIT;   //源地址传输数据宽度16位
  dma0_init_struct.DMA_SourceAddrOffset = 4;    //源地址偏移为4个字节，因为ADC0->R寄存为32位宽，A组传输完成后移动到B组
  dma0_init_struct.DMA_LastSourceAddrAdj = -8;  //主循环最后调节地址为-8个字节，因为主循环为2次计数，因此地址偏移了8个字节
  dma0_init_struct.DMA_DestAddr = (uint32)&Result;       //目的地址，即定义的结果保存数组头地址
  dma0_init_struct.DMA_DestDataSize = DMA_DST_16BIT;     //目的地址传输数据宽度16位
  dma0_init_struct.DMA_DestAddrOffset = 2;      //目的地址偏移为2个字节，因为Result为16位变量
  dma0_init_struct.DMA_LastDestAddrAdj = -4;    //目的地址最后调节地址为-4个字节
  dma0_init_struct.DMA_AutoDisableReq = FALSE;   //禁用自动禁用请求，即不受主循环计数计数限制
  //初始化DMA
  LPLD_DMA_Init(dma0_init_struct);
  //使能DMA请求
  LPLD_DMA_EnableReq(DMA_CH0);
  
  //DMA CH1，配置基本相同
  dma1_init_struct.DMA_CHx = DMA_CH1;   
  dma1_init_struct.DMA_Req = ADC1_DMAREQ;       
  dma1_init_struct.DMA_MajorLoopCnt = 2;        
  dma1_init_struct.DMA_MinorByteCnt = 2; 
  dma1_init_struct.DMA_SourceAddr = (uint32)&(ADC1->R[0]);       
  dma1_init_struct.DMA_SourceDataSize = DMA_SRC_16BIT;  
  dma1_init_struct.DMA_SourceAddrOffset = 4;
  dma1_init_struct.DMA_LastSourceAddrAdj = -8;
  dma1_init_struct.DMA_DestAddr = (uint32)&Result+4;       //目的地址，由于数组中头2个元素存ADC0的结果，因此要偏移4个字节取第3个元素的地址
  dma1_init_struct.DMA_DestDataSize = DMA_DST_16BIT;  
  dma1_init_struct.DMA_DestAddrOffset = 2;
  dma1_init_struct.DMA_LastDestAddrAdj = -4;
  dma1_init_struct.DMA_AutoDisableReq = FALSE;   
  //初始化DMA
  LPLD_DMA_Init(dma1_init_struct);
  //使能DMA请求
  LPLD_DMA_EnableReq(DMA_CH1);
    
  **********************************************
  //Step 3.配置PDB，用于触发ADC
  pdb_init_struct.PDB_CounterPeriodMs = 100;    //PDB计数器周期，这个决定了4个通道每采集一次的间隔
  pdb_init_struct.PDB_LoadModeSel = LOADMODE_0;
  pdb_init_struct.PDB_ContinuousModeEnable = TRUE;      //使能连续工作模式，即只需要开始触发一次，以后PDB就会连续工作
  pdb_init_struct.PDB_TriggerInputSourceSel = TRIGGER_SOFTWARE; //软件触发模式，即不需要用其他模块触发PDB工作
  //初始化PDB
  LPLD_PDB_Init(pdb_init_struct);
  //配置PDB预触发功能
  //使能ADC0-A组的预触发功能
  LPLD_PDB_AdcTriggerCfg(ADC0, PRETRIG_EN_A|PRETRIG_DLY_A, 0);  
  //使能ADC0-B组的预触发功能，并使用Back to Back模式
  LPLD_PDB_AdcTriggerCfg(ADC0, PRETRIG_BB_B|PRETRIG_EN_B|PRETRIG_DLY_B, 0);
  //使能ADC1-A组的预触发功能，并使用Back to Back模式
  LPLD_PDB_AdcTriggerCfg(ADC1, PRETRIG_BB_A|PRETRIG_EN_A|PRETRIG_DLY_A, 0);
  //使能ADC1-B组的预触发功能，并使用Back to Back模式
  LPLD_PDB_AdcTriggerCfg(ADC1, PRETRIG_BB_B|PRETRIG_EN_B|PRETRIG_DLY_B, 0);
  //软件触发PDB开始工作
  LPLD_PDB_SoftwareTrigger();
  } 
}
*/



