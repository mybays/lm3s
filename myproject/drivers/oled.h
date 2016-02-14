/******************************************************************
参考配套光盘上的驱动重新写一套自己的驱动,很有可能是大规模复制粘贴


*******************************************************************/
#ifndef _OLED_H_
#define _OLED_H_

//OLED清屏
extern void OLED_Clear(void);

//OLED显示字符串
extern void OLED_StringDraw(const char *pcStr,
					unsigned long ulX,
					unsigned long ulY,
					unsigned char ucLevel);

//OLED绘制图形
extern void OLED_ImageDraw( unsigned char *putImage,
					unsigned long ulX,
					unsigned long ulY,
					unsigned long ulWidth,
					unsigned long ulHeight);

//OLED绘制曲线
extern void OLED_Xplot(unsigned char *putLine);
//OLED绘制曲线,复用OLED_ImageDraw.
extern void OLED_Rewrite(unsigned char *putLine);
//OLED绘制直线
extern void OLED_LineDraw(unsigned long fX,unsigned long fY,
						unsigned long tX,unsigned long tY);
//OLED初始化
extern void OLED_Init(unsigned long ulFrequency);

//OLED使能
extern void OLED_Enable(unsigned long ulFrequency);

//OLED去能
extern void OLED_Disable(void);

//OLED打开显示
extern void OLED_DisplayOn(void);

//OLED关闭显示
extern void OLED_DisplayOff(void);


#endif
