/*
 * Peripheral Clock setup
 */

#include "stm32f407xx_spi_driver.h"

static void SPI_TXE_INTERRUPT_HANDLE(SPI_Handle_t *pSPIHandle);
static void SPI_RXNE_INTERRUPT_HANDLE(SPI_Handle_t *pSPIHandle);
static void SPI_OVR_INTERRUPT_HANDLE(SPI_Handle_t *pSPIHandle);

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_EN();
			}else if (pSPIx == SPI2)
			{
				SPI2_PCLK_EN();
			}else if (pSPIx == SPI3)
			{
				SPI3_PCLK_EN();
			}else if (pSPIx == SPI4)
			{
				SPI4_PCLK_EN();
			}
		}
		else
		{
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_DI();
			}else if (pSPIx == SPI2)
			{
				SPI2_PCLK_DI();
			}else if (pSPIx == SPI3)
			{
				SPI3_PCLK_DI();
			}else if (pSPIx == SPI4)
			{
				SPI4_PCLK_DI();
			}
		}
}
/*
 * Init and DeInit
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	uint32_t temp = 0;

	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//1.Configure Device Mode
	temp |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//2.Configure Bus  Configuration
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//BIDI Mode Cleared
		temp &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//BIDI Mode should be set
		temp |= (1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//BIDI Mode is cleared
		temp &= ~(1 << SPI_CR1_BIDIMODE);
		//RXonly is to be set
		temp |= (1 << SPI_CR1_RXONLY);
	}

	//3.configure Clock speed
	temp |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4.COnfigure Data Frame format
	temp |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//5.Configure Clock Polarity
	temp |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6.Configure Clock Phase
	temp |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	//7.Configure Software slave Management
	temp |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 = temp;
}
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}else if (pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}else if (pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
}
/*
 * Data Send and Receive
 */

void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//1.Wait Until TXE is set
		while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG) == FLAG_RESET);

		//2.Check the DFF bit in CR1
		if((pSPIx->CR1 & (1 << SPI_CR1_DFF)) == SPI_DFF_8BITS)
		{
			//8-Bit Data Load into Data Register
			//1.Load data into DR
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
		else
		{
			//16-Bit data load into Data Register
			//2.Load data into DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
			(uint16_t*)pTxBuffer++;
		}
	}
}
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//1.Wait Until RXNE is set
		while(SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG) == FLAG_RESET);

		//2.Check the DFF bit in CR1
		if((pSPIx->CR1 & (1 << SPI_CR1_DFF)) == SPI_DFF_8BITS)
		{
			//8-Bit Data Load from DR
			//1.Load data from DR to pRxBuffer
			*pRxBuffer = pSPIx->DR ;
			Len--;
			pRxBuffer++;
		}
		else
		{
			//16-Bit data load into Data Register
			//2.Load data from DR to pRxBuffer
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			Len--;
			Len--;
			(uint16_t*)pRxBuffer++;
			(uint16_t*)pRxBuffer++;
		}
	}
}

/*
 * Interrupt Send and Receive Data.
 */

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t txstate = pSPIHandle->TxState;
	if(txstate != SPI_BUSY_IN_TX)
	{
		//fetch the state, of the SPI
		pSPIHandle->TxState = SPI_BUSY_IN_TX;
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		//Implement the Code to Enable the Interrupt caused due to TXE
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_TXEIE);
	}
	return txstate;
}
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t rxstate = pSPIHandle->RxState;
	if(rxstate != SPI_BUSY_IN_RX)
	{
		pSPIHandle->RxState = SPI_BUSY_IN_RX;
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		//implement the Code to Enable the Interrupt Caused due to RXNE
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
	}
	return rxstate;
}

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp1,temp2;
	//Check for the TXE interrupt
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);

	//Check for the TXEIE
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if(temp1 && temp2)
	{
		//Interrupt Caused Due to Transmission
		SPI_TXE_INTERRUPT_HANDLE(pSPIHandle);
	}

	//Check for the RXE interrupt
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);

	//Check for RXNEIE
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if(temp1 && temp2)
	{
		//Interrupt id Caused due to Receiving
		SPI_RXNE_INTERRUPT_HANDLE(pSPIHandle);
	}

	//Check for the OVR interrupt
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);

	//Check for ERRIE
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);
	if(temp1 && temp2)
	{
		//Interrupt is Caused due to Over Run
		SPI_OVR_INTERRUPT_HANDLE(pSPIHandle);
	}
}
static void SPI_TXE_INTERRUPT_HANDLE(SPI_Handle_t *pSPIHandle)
{
	//1.Check the DFF bit in CR1
	if((pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) == SPI_DFF_8BITS)
	{
		//8-Bit Data Load into Data Register
		//1.Load data into DR
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}
	else
	{
		//16-Bit data load into Data Register
		//2.Load data into DR
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}

	if(!(pSPIHandle->TxLen))
	{
		//TxLen is Zero and inform the application that the transmission
		//is over
		//Clear the TXEIE flag and Close Transmission
		pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
		pSPIHandle->pTxBuffer = NULL;
		pSPIHandle->TxLen = 0;
		pSPIHandle->TxState = SPI_READY;

		//Event Call Back Application
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
	}
}
static void SPI_RXNE_INTERRUPT_HANDLE(SPI_Handle_t *pSPIHandle)
{
	//1.Check the DFF bit in CR1
	if((pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) == SPI_DFF_8BITS)
	{
		//8-Bit Data Load from DR
		//1.Load data from DR to pRxBuffer
		*(pSPIHandle->pRxBuffer) = (uint8_t)pSPIHandle->pSPIx->DR ;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer--;
	}
	else
	{
		//16-Bit data load into Data Register
		//2.Load data from DR to pRxBuffer
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;;
		pSPIHandle->RxLen--;;
		pSPIHandle->pRxBuffer++;
		pSPIHandle->pRxBuffer++;
	}


	if(!(pSPIHandle->RxLen))
	{
		//RxLen is Zero and inform the application that the Receiving
		//is over
		//Clear the RXNEIE flag and Close Reception(You can write separate Function for this if necessary)
		pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
		pSPIHandle->pRxBuffer = NULL;
		pSPIHandle->RxLen = 0;
		pSPIHandle->RxState = SPI_READY;

		//Event Call Back Application
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
	}

}
static void SPI_OVR_INTERRUPT_HANDLE(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp = 0;
	//1. Clear the OVR Flag
	if(pSPIHandle->TxState == SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	//2.Inform the Application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

/*
 * IRQ Configuration and ISR Handling
 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			if(IRQNumber <= 31)
			{
				//program ISER0 register
				*NVIC_ISER0 |= ( 1 << IRQNumber );

			}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
			{
				//program ISER1 register
				*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
			}
			else if(IRQNumber >= 64 && IRQNumber < 96 )
			{
				//program ISER2 register //64 to 95
				*NVIC_ISER3 |= ( 1 << (IRQNumber % 64) );
			}
		}else
		{
			if(IRQNumber <= 31)
			{
				//program ICER0 register
				*NVIC_ICER0 |= ( 1 << IRQNumber );
			}
			else if(IRQNumber > 31 && IRQNumber < 64 )
			{
				//program ICER1 register
				*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
			}
			else if(IRQNumber >= 6 && IRQNumber < 96 )
			{
				//program ICER2 register
				*NVIC_ICER3 |= ( 1 << (IRQNumber % 64) );
			}
		}
}
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx,uint8_t FlagName)
{
	if(pSPIx->SR & (1 << FlagName))
	{
		return FLAG_SET;
	}
	else
	{
		return FLAG_RESET;
	}
}
__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{
	//Weak Implementation of Application Event CallBack Function
}


