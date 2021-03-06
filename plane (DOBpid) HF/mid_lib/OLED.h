#ifndef _OLED_H_
#define _OLED_H_

#define X_WIDTH 128
#define Y_WIDTH 64
/************************************************************************
     OLED 一行可以显示16个字符，可以显示8行。
************************************************************************/

//#define OLED_SCL=1   LPLD_GPIO_Output_b(PTA,7,1)
//#define OLED_SDA=1   LPLD_GPIO_Output_b(PTA,6,1)
//#define OLED_RST=1   LPLD_GPIO_Output_b(PTE,28,1)
//#define OLED_DC=1   LPLD_GPIO_Output_b(PTE,27,1)
//#define OLED_CS=1   LPLD_GPIO_Output_b(PTE,26,1)

extern uint8 mushroom[];
extern uint8 sky1[];
extern uint8 sky2[];
extern uint8 sky3[];
#define XLevelL		0x00
#define XLevelH		0x10
#define XLevel		((XLevelH&0x0F)*16+XLevelL)
#define Max_Column	128
#define Max_Row		64
#define	Brightness	0xCF
void OLEDIO_init();
extern uint8 lanzhou96x64[768];
void OLED_Init(void);
void OLED_CLS(void);
void OLED_P6x8Str(uint8 x,uint8 y,uint8 ch[]);
void OLED_P8x16Str(uint8 x,uint8 y,uint8 ch[]);
void OLED_P14x16Str(uint8 x,uint8 y,uint8 ch[]);
void OLED_Print(uint8 x, uint8 y, uint8 ch[]);
void OLED_PutPixel(uint8 x,uint8 y);
void OLED_Rectangle(uint8 x1,uint8 y1,uint8 x2,uint8 y2,uint8 gif);
void OLED_Set_Pos(uint8 x, uint8 y);
void OLED_WrDat(uint8 data);
void Draw_LibLogo(void);
void Draw_Landzo(void);
void Draw_BMP(uint8 x0,uint8 y0,uint8 x1,uint8 y1,uint8 bmp[]);
void OLED_Fill(uint8 dat);
void Dly_ms(uint16 ms);
void OLED_P6x8Char(uint8 ucIdxX, uint8 ucIdxY, uint8 ucData);
void OLED_PrintFloat(uint8 ucIdxX, uint8 ucIdxY, float64 sData);


#endif 