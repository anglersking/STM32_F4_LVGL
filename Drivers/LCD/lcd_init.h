#ifndef __LCD_INIT_H
#define __LCD_INIT_H

#include "stm32f4xx.h"
#define u16 uint16_t
#define  u8 uint8_t
#define  u32 uint32_t

#define USE_HORIZONTAL 2  //���ú�������������ʾ 0��1Ϊ���� 2��3Ϊ����


#if USE_HORIZONTAL==0||USE_HORIZONTAL==1
#define LCD_W 172
#define LCD_H 320

#else
#define LCD_W 320
#define LCD_H 172
#endif



//-----------------LCD�˿ڶ���---------------- 

#define LCD_SCLK_Clr() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET)
#define LCD_SCLK_Set() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET)

#define LCD_MOSI_Clr() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET)//SDA=MOSI
#define LCD_MOSI_Set() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET)

#define LCD_RES_Clr()  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET)//SDA=MOSI
#define LCD_RES_Set()  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET)//SDA=MOSI

#define LCD_DC_Clr()   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET)//SDA=MOSI
#define LCD_DC_Set()   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET)//SDA=MOSI
 		     
#define LCD_CS_Clr()   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET)//SDA=MOSI
#define LCD_CS_Set()  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET)//SDA=MOSI

#define LCD_BLK_Clr()  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET)//SDA=MOSI
#define LCD_BLK_Set()  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET)//SDA=MOSI




void LCD_GPIO_Init(void);//��ʼ��GPIO
void LCD_Writ_Bus(u8 dat);//ģ��SPIʱ��
void LCD_WR_DATA8(u8 dat);//д��һ���ֽ�
void LCD_WR_DATA(u16 dat);//д�������ֽ�
void LCD_WR_REG(u8 dat);//д��һ��ָ��
void LCD_Address_Set(u16 x1,u16 y1,u16 x2,u16 y2);//�������꺯��
void LCD_Init(void);//LCD��ʼ��
#endif




