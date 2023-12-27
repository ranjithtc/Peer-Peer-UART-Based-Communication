/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Nov 16, 2022
 *      Author: Ranjith
 */


#include "stm32f407xx_gpio_driver.h"
#include <stdint.h>
/*
 * Peripheral Clock setup
 */

/**************************************@fn- GPIO_PeriClockControl***************************/

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DI();
		}
	}
}
/*
 * Init and DeInit
 */
/**************************************@fn- GPIO_Init**************************************/

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;

	//Enable the peripheral Clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);
	//1 Configure mode

	//pGPIOHandle->pGPIOx->MODER = 0x00000000; //This is bad, clears out modes for other pins every call


	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//the non-interrupt mode
		temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 <<  (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
		pGPIOHandle->pGPIOx->MODER |= temp;

	}else
	{
		if( pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber == GPIO_MODE_IT_FT )
		{
			// 1. configure the FTSR
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			// Clear RTSR bit
			EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );

		}else if( pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber == GPIO_MODE_IT_RT )
		{
			// 1. configure the RTSR
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			// Clear RTSR bit
			EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		}else if( pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber == GPIO_MODE_IT_RFT )
		{
			// 1. configure both
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			// Clear RTSR bit
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		}

		//2. configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);

		//3. enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	}


	temp = 0;

	//2 Configure speed
	temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 <<  (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) ); //instructor has 1 not 2
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;


	temp = 0;

	//3 Configure pupd settings
	temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 <<  (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );//instructor has 1 not 2
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;

	//4 configure the optype
	temp = (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) & ( pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 <<  (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ) );
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp = 0;

	//5 configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		//configure alt function registers
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << ( 4 * temp2 ) );
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4 * temp2 ) );
	}
}

	/******************************************@fn- GPIO_DeInit*****************************/

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{

	if(pGPIOx == GPIOA)
			{
				GPIOA_REG_RESET();
			}else if (pGPIOx == GPIOB)
			{
				GPIOB_REG_RESET();
			}else if (pGPIOx == GPIOC)
			{
				GPIOC_REG_RESET();
			}else if (pGPIOx == GPIOD)
			{
				GPIOD_REG_RESET();
			}else if (pGPIOx == GPIOE)
			{
				GPIOE_REG_RESET();
			}else if (pGPIOx == GPIOF)
			{
				GPIOF_REG_RESET();
			}else if (pGPIOx == GPIOG)
			{
				GPIOG_REG_RESET();
			}else if (pGPIOx == GPIOH)
			{
				GPIOH_REG_RESET();
			}else if (pGPIOx == GPIOI)
			{
				GPIOI_REG_RESET();
			}
}

/*
 * Read and Write
 */

/*****************************************@fn- GPIO_ReadFromInputPin*********************/

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;

	value = (uint8_t) (( pGPIOx->IDR >> PinNumber ) & 0x00000001 );

	return value;

}

/******************************************@fn- GPIO_ReadFromInputPort*************************/

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{

	uint16_t value;

	value = pGPIOx->IDR;

	return value;
}

/**********************************@fn- GPIO_WriteToOutputPin********************************/

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{

	if(Value == GPIO_PIN_SET)
	{
		//write 1 to the utput data register at the bit field corresponding to the pin number
		pGPIOx->ODR |= ( 1 << PinNumber);
	}else
	{
		//write 0
		pGPIOx->ODR &= ~( 1 << PinNumber);
	}
}

/********************************************************************************************
 * @fn						- GPIO_WriteToOutputPort
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;

}

/********************************************************************************************
 * @fn						- GPIO_ToggleOutputPin
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= ( 1 << PinNumber );
}

/*
 * IRQ Config and ISR Handling
 */

/********************************************************************************************
 * @fn						- GPIO_IRQInterruptConfig
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	//processor-specific config (from Cortex-M4 Generic User guide - Table 4.2 NVIC Registers)
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber < 64 )
		{
			//program ISER1
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );

		}else if(IRQNumber < 96 )
		{
			//program ISER2
			*NVIC_ISER2 |= ( 1 << (IRQNumber % 64) );

		}

	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0
			*NVIC_ICER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber < 64 )
		{
			//program ICER1
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );

		}else if(IRQNumber < 96 )
		{
			//program ICER2
			*NVIC_ICER2 |= ( 1 << (IRQNumber % 64) );

		}
	}

}


/********************************************************************************************
 * @fn						- GPIO_IRQPriorityConfig
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. first find out the IPR register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = ( 8 * iprx_section ) + ( 8 - NO_PR_BITS_IMPLEMENTED );
	*( NVIC_PR_BASE_ADDR + iprx ) |= ( IRQPriority << shift_amount ); //removed the multiply by 4 since 32-bit pointer is already incremented in blocks of 4bytes each increment
}

/********************************************************************************************
 * @fn						- GPIO_IRQHandling
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear the exti pr register corresponding to the pin number
	if(EXTI->PR & (1 << PinNumber))
	{
		//Clear
		EXTI->PR |= (1 << PinNumber);
	}
}

