/***
***图像处理C文件
***李锐戈
*2015 4 1
*/


#include "common.h"
#include "include.h"

uint8 blackline[PHOTO_SIZE];
uint16 aim_X = 0;
uint16 aim_Y = 0;


void find_blackblock_h()
{
  //二值化处理
  uint16 i,j,k;
  
  uint32 sum = 0;
  uint8 image_data_1[V];
  uint8 image_data_2[V];
  uint8 H_error[V] = {0,0};
  uint8 V_error[2] = {0,0};
  uint8 result[V] ;
  for (j = 0 ; j < V ; j++)
  {
    for ( i = 0; i < H; i++)
    {
      if ( BUFF[j * H + i] < THR ) //黑线
      {
        blackline[ j * H + i ] = 255;
      }
      else
      {
        blackline[ j * H + i ] = 0;
      }
    }
  }
  /*   for(i = 140 ;i<230 ; i++)
  {
  LPLD_UART_PutChar(UART0,i);
  LPLD_UART_PutChar(UART0,i);
  for (j = 0;j < H;j++)
  {
  LPLD_UART_PutChar(UART0,blackline[i * H + j]);
}
}
  */
  
  
  //水平寻找黑块
  //打圆心靶
  //应该返回横竖两个偏移量
  
  
  //初始化
  for ( i = 0 ; i < V; i++ )
  {
    image_data_1[i] = 0;
    image_data_2[i] = 0;
    H_error[i] = 0;
    result[i] = 0;
    k = 0;
  }
  
  for ( i = 10 ; i< V - 10;i++  )//行扫描 ,
  {
    for ( j = 20;j< H-20;j++ )//每行逐点判断
    {
      /******上下边缘判断**********/
      if ( ( blackline [ i*H + j ] == 255 ) 
          &&  ( blackline [ i*H + j +1 ] == 255 )
            &&  ( blackline [ i*H + j +2 ] == 255)
              &&  ( blackline [ i*H + j +3 ] == 255)
                &&  ( blackline [ i*H + j +4 ] == 255)
                  &&  ( blackline [ i*H + j +5 ] == 255 ) )
      {
        if ( V_error[0] == 0 )
        {
          V_error[0] = i;
        }
        else
        {
          V_error[1] = i;
        }
      }
      
      /*******************左右边缘判断************************************/    
      if ( ( blackline [ i*H + j ] == 255 ) 
          &&  ( blackline [ i*H + j +1 ] == 255 )
            /* && (  blackline [ i*H + j + 2 ] == 255 )*/) //出现三个连续黑点，认为是一个黑块
      {
        image_data_1[i] = j;
        break;
      }
      else //不是三个连续黑点
      {;}
    }
    for ( j = H - 20;j> 20;j-- )//每行逐点判断
    {
      if ( ( blackline [ i*H + j ] == 255 ) 
          &&  ( blackline [ i*H + j -1 ] == 255 )
            /*&& (  blackline [ i*H + j - 2 ] == 255 )*/) //出现三个连续黑点，认为是一个黑块
      {
        image_data_2[i] = j;
        break;
      }
    }
    
    
    if (image_data_1[i] > 10 && image_data_1[i] < 130 && image_data_2[i] > 10 && image_data_2[i] < 130)
    {
      H_error[i] = image_data_1[i] + ( image_data_2[i] - image_data_1[i] ) /2;
      sum += H_error[i];
      k++;
    }
  }
  
  aim_X = sum / k;
  aim_Y = V_error[0] + ( V_error[1] - V_error[0] ) / 2;
  
  //sendRoadDataToCamViewer(image_data_1);
  //sendRoadDataToCamViewer(image_data_2);
  //sendRoadDataToCamViewer(H_error);
  //sendRoadDataToCamViewer(result);
  //push(0,aim_X);
  //push(1,aim_Y);
  //sendDataToScope();
}



/****
找光斑
***/
uint16 shot_X = 0;
uint16 shot_Y = 0;
void find_spot()
{
  //第二阈值二值化
  uint16 i,j;
  uint8 cnt = 0;
  for (j = 0 ; j < V ; j++)
  {
    for ( i = 0; i < H; i++)
    {
      if ( BUFF[j * H + i] < 252) //亮斑
      {
        blackline[ j * H + i ] = 255;
      }
      else
      {
        blackline[ j * H + i ] = 0;
      }
    }
  }
  
  for ( j = 1;j < V - 1; j++ )
  {
    for ( i = 1;i < H -1 ; i++ )
    {
      if ( blackline [j * H + i] == 0 && blackline [j * H + i + 1] == 0 )
      {
        cnt = 2;
        if (blackline[ ( j - 1 ) * H +i - 1 ] == 0 )
        { cnt ++; }
        if (blackline[ ( j - 1 ) * H +i ] == 0 )
        { cnt ++; }
        if ( blackline[ ( j - 1 ) * H +i + 1 ] == 0)
        { cnt ++; }
        if (blackline[ ( j + 1 ) * H + i - 1 ] == 0 )
        { cnt ++; }
        if (blackline[ ( j + 1 ) * H + i ] == 0 )
        { cnt ++; }
        if ( blackline[ ( j +1 ) * H + i + 1 ] == 0)
        { cnt ++; }
      }
      if ( cnt >= 6 )
      {
        shot_X = i;
        shot_Y = j;
        goto _break;
      }
      else
      {
        
      }
      
    }
  }
_break:
  //push(3,shot_X);
  //push(4,shot_Y);
  ;
  
}


/*for (  i = 10 ; i<340;i++   )
{
if ( H_error[i] != 0 && H_error[i] != 255 && H_error[i + 1] != 0 && H_error[i +1] != 255)
{
V_error[0] = i;
    }   
  }
for (  i = 340 ; i>10;i-- )
{
if ( H_error[i] != 0 && H_error[i] != 255 && H_error[i - 1 ] != 0 && H_error[i - 1] != 255)
{
V_error[1] = i;
    }   
  }
result [V_error[0] + ( V_error[1] - V_error[0] ) / 2] = sum / k;
*/











