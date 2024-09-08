#include "LCD16x2.h"
#include "string.h"
#include "stdio.h"

void LCD_Write(uint8_t data)
{
	HAL_GPIO_WritePin(PORT, PIN4, ((data >> 0) & 0x01));
	HAL_GPIO_WritePin(PORT, PIN5, ((data >> 1) & 0x01));
	HAL_GPIO_WritePin(PORT, PIN6, ((data >> 2) & 0x01));
	HAL_GPIO_WritePin(PORT, PIN7, ((data >> 3) & 0x01));

	HAL_GPIO_WritePin(PORT,ENABLE, 1);
	HAL_Delay(1);
	HAL_GPIO_WritePin(PORT,ENABLE, 0);
}

//Thanh ghi DATA=0 CMT=1
void LCD_Send(uint8_t Reg, uint8_t data)
{
	HAL_GPIO_WritePin(PORT, RESET, Reg);
	LCD_Write(data >> 4);
	LCD_Write(data);
}

//START LCD
void LCD_Init()
{
	HAL_GPIO_WritePin(PORT, READWRITE, 0);

	LCD_Send(cmd_reg, 0x33);                   // lenh de khoi tao
	LCD_Send(cmd_reg, 0x32);                  // lenh de khoi tao
	LCD_Send(cmd_reg, 0x28);                   // che do 4 bit, 2 hang, 5x7
	LCD_Send(cmd_reg, 0x0C);                   // hien thi man hinh va tat con tro
	LCD_Send(cmd_reg, 0x06);                  // tang con tro
	LCD_Send(cmd_reg, 0x01);                   // xoa toan man hinh
}

//CLEAR LCD
void LCD_Clear()                       
{
	LCD_Send(cmd_reg, 0x01);
}

//SELECT POSITION
void LCD_Location(uint8_t x, uint8_t y)
{
  if(x == 0)
	  LCD_Send(cmd_reg, 0x80 + y);
  else if(x == 1)
	  LCD_Send(cmd_reg, 0xC0 + y);
}

//WRITE STRING LCD
void LCD_Write_String(char* string)          
{
	for(uint8_t i = 0; i < strlen(string); i++)
	{
		LCD_Send(data_reg, string[i]);
	}
}

//WRITE NUMBER LCD
void LCD_Write_Number(int number)                 // ghi chu so
{
	char buffer[8];
	sprintf(buffer, "%d", number);
	LCD_Write_String(buffer);
}
