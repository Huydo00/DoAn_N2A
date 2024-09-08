#ifndef __LCD16X2_H__
#define __LCD16X2_H__

#include "stm32f1xx_hal.h"

#define PORT    GPIOA

#define RESET      GPIO_PIN_1
#define READWRITE      GPIO_PIN_2
#define ENABLE       GPIO_PIN_3

#define PIN4      GPIO_PIN_4
#define PIN5      GPIO_PIN_5
#define PIN6      GPIO_PIN_6
#define PIN7      GPIO_PIN_7

#define cmd_reg     0
#define data_reg    1

void LCD_Write(uint8_t data);
void LCD_Send(uint8_t Reg, uint8_t data);
void LCD_Init();
void LCD_Clear();
void LCD_Location(uint8_t x, uint8_t y);
void LCD_Write_String(char* string);
void LCD_Write_Number(int number);

#endif
