/***
***图像处理H文件
***李锐戈
*2015 4 1
*/

#ifndef _IMAGE_H_
#define _IMAGE_H_

#define THR  52
extern uint8 blackline[PHOTO_SIZE];
extern uint16 aim_X,aim_Y,shot_X,shot_Y;

void find_blackblock_h();
void find_spot();

#endif