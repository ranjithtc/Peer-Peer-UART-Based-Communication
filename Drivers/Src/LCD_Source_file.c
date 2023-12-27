/*
 * LCD_Source_file.c
 *
 *  Created on: Nov 16, 2022
 *      Author: Ranjith
 */
#include "LCD_Header_file.h"

void delayMs(uint32_t n)
{
	uint32_t i;
	for(; n>0;n--)
		for(i = 0;i<3195;i++);
}

void LCD_Data(char data)
{
	//Register Select is set to 1 (Select data Register Allows User to send Data)
	GPIO_WriteToOutputPin(GPIOC, 0, 1);

	//R/W Allows user to read or write to LCD for Reading 1 for writing 0
	GPIO_WriteToOutputPin(GPIOC, 1, 0);

	//Calls the Character to ascii value converter function
	Char_to_Binary(data);

	//Setting Enable pin to High
	GPIO_WriteToOutputPin(GPIOC, 2, 1);

	delayMs(10);

	//Enable Pin to Low
	GPIO_WriteToOutputPin(GPIOC, 2, 0);

	delayMs(20);
}

void LCD_Init()
{
	PORTS_Init();			//Ports Initialization

	LCD_command(0x1);		//Clear Display Screen

	LCD_command(0x38);		//Mode 2 lines

	LCD_command(0xF);		//Display On And Cursor Blinking

	LCD_command(0x80);		//Force Cursor to Beginning of first Line
}
void PORTS_Init()
{
	//RCC Clock Enable for PORT D and PORT C
	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_PeriClockControl(GPIOC, ENABLE);

	//Data Ports from DB0 to DB7

	GPIO_Handle_t gpioPD;
	gpioPD.pGPIOx = GPIOD;
	gpioPD.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	gpioPD.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioPD.GPIO_PinConfig.GPIO_PinMode   = GPIO_MODE_OUT;
	gpioPD.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	gpioPD.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GPIO_Init(&gpioPD);

	gpioPD.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
	GPIO_Init(&gpioPD);

	gpioPD.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
	GPIO_Init(&gpioPD);

	gpioPD.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GPIO_Init(&gpioPD);

	gpioPD.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
	GPIO_Init(&gpioPD);

	gpioPD.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIO_Init(&gpioPD);

	gpioPD.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&gpioPD);

	gpioPD.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&gpioPD);

	//Ports for Register Select,Read/Write,Enable

	GPIO_Handle_t gpioPC;
	gpioPC.pGPIOx = GPIOC;
	gpioPC.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	gpioPC.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioPC.GPIO_PinConfig.GPIO_PinMode   = GPIO_MODE_OUT;
	gpioPC.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	gpioPC.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GPIO_Init(&gpioPC);

	gpioPC.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
	GPIO_Init(&gpioPC);

	gpioPC.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
	GPIO_Init(&gpioPC);
}


void LCD_command(unsigned char data)
{

	int binary = 0;
	GPIO_WriteToOutputPin(GPIOC, 0, 0);		//Select Command Register RS = 0

	for(uint8_t i = 0; i < 8 ;i++)
	{
		binary = data % 2;
		data = data/2;
		GPIO_WriteToOutputPin(GPIOD, i, binary);
	}

	GPIO_WriteToOutputPin(GPIOC, 2, 1);		//Setting Enable Pin High

	delayMs(10);	//Delay for 10 ms

	GPIO_WriteToOutputPin(GPIOC, 2, 0);		//Enable Pin to Low

	delayMs(20);	//Delay for 20 ms

}

//Function to convert Character to Binary

void Char_to_Binary(char data)
{
	int bin;
	char ch = data;
	for (uint32_t i = 0; i < 8; i++)
	{
		bin = ((ch << i) & 0x80) ? 1 : 0;
		GPIO_WriteToOutputPin(GPIOD, (7-i), bin);
		bin = 0;
	}
}





