#include <stdio.h>
#include <stdint.h>
#include "stm32f407xx.h"
#include "stm32f407xx_usart_driver.h"

//Function Definitions used in the main program
void GPIO_ButtonInit();
void USART2_GPIOInit();
void USART2_Init();
void delay();
uint8_t Keypad_Btn_Press();
void USART2_IRQHandler(void);
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t event);

//Global Structure
USART_Handle_t usart2_handle;

//Global Variables
uint8_t mssg[1];
uint8_t BtnPrsd = 0;
uint8_t msg[1];

int main(void)
{
	USART_IRQInterruptConfig(IRQ_NO_USART2, ENABLE);
	USART_IRQPriorityConfig(IRQ_NO_USART2, 0);
	uint8_t msg1[1];
	LCD_Init();				//LCD Initialization
	USART2_Init();			//USART Initialization
	USART2_GPIOInit();		//USART to GPIO Initialization
	GPIO_ButtonInit();		//Initialization of pins for Keypad
	USART_PeriClockControl(USART2, ENABLE);
	USART_PeripheralControl(USART2, ENABLE);


	USART_ReceiveDataIT(&usart2_handle, (uint8_t *)msg, 1);//UART2 Receive Interrupt

	while(1)
	{
		mssg[0] = Keypad_Btn_Press();	//Detect Keypad Button Press.

		if(BtnPrsd == 1)	//Transmit the data if Keypad is Pressed.
		{
			for(uint8_t i = 0;i < 1; i++)
			{
				GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_12, SET);
				msg1[0] = mssg[i];
				USART_SendData(&usart2_handle, (uint8_t*)msg1, 1);	//Use of USART to Transmit the Data Received from Keypad.
				delay();
				GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_12, RESET);
				BtnPrsd = 0;
			}
		}
	}
}
void delay()
{
	for(uint32_t i = 0; i < 450000; i++);
}
void GPIO_ButtonInit()
{
	//Clock Enable for GPIOD peripheral
	GPIO_PeriClockControl(GPIOD, ENABLE);

	//GPIO Initialization of Port D pin 12 for Green Led Configuration
	GPIO_Handle_t gpioled;

	gpioled.pGPIOx = GPIOD;
	gpioled.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioled.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	gpioled.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioled.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	gpioled.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Init(&gpioled);

	//Pin Configuration for PD0 to PD3 as Output Mode connected to R1-R4 of Keypad

	GPIO_Handle_t gpioPDR;
	gpioPDR.pGPIOx = GPIOD;
	gpioPDR.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioPDR.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioPDR.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	gpioPDR.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioPDR.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIO_Init(&gpioPDR);

	gpioPDR.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
	GPIO_Init(&gpioPDR);

	gpioPDR.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
	GPIO_Init(&gpioPDR);

	gpioPDR.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GPIO_Init(&gpioPDR);

	//Pin Configuration for PD8 to PD11 as Input Mode with Pull resistors connected to C1 to C4 of Keypad

	GPIO_Handle_t gpioPDC;
	gpioPDC.pGPIOx = GPIOD;
	gpioPDC.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpioPDC.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioPDC.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	gpioPDC.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioPDC.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GPIO_Init(&gpioPDC);

	gpioPDC.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	GPIO_Init(&gpioPDC);

	gpioPDC.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
	GPIO_Init(&gpioPDC);

	gpioPDC.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_11;
	GPIO_Init(&gpioPDC);


}
void USART2_GPIOInit()
{
	GPIO_PeriClockControl(GPIOA, ENABLE);

	//For Transmission
	GPIO_Handle_t usart_gpioPA;
	usart_gpioPA.pGPIOx = GPIOA;
	usart_gpioPA.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	usart_gpioPA.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	usart_gpioPA.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	usart_gpioPA.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	usart_gpioPA.GPIO_PinConfig.GPIO_PinAltFunMode = 7;
	usart_gpioPA.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
	GPIO_Init(&usart_gpioPA);

	//For Receiver
	usart_gpioPA.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GPIO_Init(&usart_gpioPA);
}
void USART2_Init()
{
	usart2_handle.pUSARTx = USART2;
	usart2_handle.USART_Config.USART_Baud = USART_STD_BAUD_1200;
	usart2_handle.USART_Config.USART_Mode = USART_MODE_TXRX;
	usart2_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	usart2_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	usart2_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	usart2_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART_Init(&usart2_handle);
}
uint8_t Keypad_Btn_Press()
{
	//Make all rows High
	GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_0, SET);
	GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_1, SET);
	GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_2, SET);
	GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_3, SET);

	//Make R1 low
	GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_0, RESET);

	//Scanning for C1 Column
	if(!(GPIO_ReadFromInputPin(GPIOD, GPIO_PIN_NO_8)))
	{
		delay();
		mssg[0] = '1';
		BtnPrsd = 1;
	}
	//Scanning for C2 Column
	if(!(GPIO_ReadFromInputPin(GPIOD, GPIO_PIN_NO_9)))
	{
		delay();
		mssg[0] = '2';
		BtnPrsd = 1;
	}
	//Scanning for C3 Column
	if(!(GPIO_ReadFromInputPin(GPIOD, GPIO_PIN_NO_10)))
	{
		delay();
		mssg[0] = '3';
		BtnPrsd = 1;
	}
	//Scanning for C4 Column
	if(!(GPIO_ReadFromInputPin(GPIOD, GPIO_PIN_NO_11)))
	{
		delay();
		mssg[0] = 'A';
		BtnPrsd = 1;
	}

	//Make All rows High
	GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_0, SET);
	GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_1, SET);
	GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_2, SET);
	GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_3, SET);

	//Make R2 Row Low
	GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_1, RESET);

	//Scanning for C1 Column
	if(!(GPIO_ReadFromInputPin(GPIOD, GPIO_PIN_NO_8)))
	{
		delay();
		mssg[0] = '4';
		BtnPrsd = 1;
	}
	//Scanning for C2 Column
	if(!(GPIO_ReadFromInputPin(GPIOD, GPIO_PIN_NO_9)))
	{
		delay();
		mssg[0] = '5';
		BtnPrsd = 1;
	}
	//Scanning for C3 Column
	if(!(GPIO_ReadFromInputPin(GPIOD, GPIO_PIN_NO_10)))
	{
		delay();
		mssg[0] = '6';
		BtnPrsd = 1;
	}
	//Scanning for C4 Column
	if(!(GPIO_ReadFromInputPin(GPIOD, GPIO_PIN_NO_11)))
	{
		delay();
		mssg[0] = 'B';
		BtnPrsd = 1;
	}

	//Make All rows High
	GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_0, SET);
	GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_1, SET);
	GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_2, SET);
	GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_3, SET);

	//Make R3 Row Low
	GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_2, RESET);

	//Scanning for C1 Column
	if(!(GPIO_ReadFromInputPin(GPIOD, GPIO_PIN_NO_8)))
	{
		delay();
		mssg[0] = '7';
		BtnPrsd = 1;
	}
	//Scanning for C2 Column
	if(!(GPIO_ReadFromInputPin(GPIOD, GPIO_PIN_NO_9)))
	{
		delay();
		mssg[0] = '8';
		BtnPrsd = 1;
	}
	//Scanning for C3 Column
	if(!(GPIO_ReadFromInputPin(GPIOD, GPIO_PIN_NO_10)))
	{
		delay();
		mssg[0] = '9';
		BtnPrsd = 1;
	}
	//Scanning for C4 Column
	if(!(GPIO_ReadFromInputPin(GPIOD, GPIO_PIN_NO_11)))
	{
		delay();
		mssg[0] = 'C';
		BtnPrsd = 1;
	}

	//Make All rows High
	GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_0, SET);
	GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_1, SET);
	GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_2, SET);
	GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_3, SET);

	//Make R3 Row Low
	GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_3, RESET);

	//Scanning for C1 Column
	if(!(GPIO_ReadFromInputPin(GPIOD, GPIO_PIN_NO_8)))
	{
		delay();
		mssg[0] = '*';
		BtnPrsd = 1;
	}
	//Scanning for C2 Column
	if(!(GPIO_ReadFromInputPin(GPIOD, GPIO_PIN_NO_9)))
	{
		delay();
		mssg[0] = '0';
		BtnPrsd = 1;
	}
	//Scanning for C3 Column
	if(!(GPIO_ReadFromInputPin(GPIOD, GPIO_PIN_NO_10)))
	{
		delay();
		mssg[0] = '#';
		BtnPrsd = 1;
	}
	//Scanning for C4 Column
	if(!(GPIO_ReadFromInputPin(GPIOD, GPIO_PIN_NO_11)))
	{
		delay();
		mssg[0] = 'D';
		BtnPrsd = 1;
	}
	return mssg[0];
}
void USART2_IRQHandler(void)
{
	USART_IRQHandling(&usart2_handle);
	//USART_ReceiveData(&usart2_handle, (uint8_t *)msg, 1);
	//LCD_Data((uint8_t)*msg);
	//delayMs(200);
}
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t event)
{
	if( event == USART_EVENT_RX_CMPLT)
	{
		LCD_Data((uint8_t)*msg);
		delayMs(200);
	}
}
