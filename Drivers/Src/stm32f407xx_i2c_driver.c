#include "stm32f407xx_i2c_driver.h"

static void I2C_ClearAddrFlag(I2C_Handle_t *pI2CHandle);
/*
 * Peripheral Clock Enable or Disable
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			if(pI2Cx == I2C1)
			{
				I2C1_PCLK_EN();
			}else if (pI2Cx == I2C2)
			{
				I2C2_PCLK_EN();
			}else if (pI2Cx == I2C3)
			{
				I2C3_PCLK_EN();
			}
		}
		else
		{
			if(pI2Cx == I2C1)
			{
				I2C1_PCLK_DI();
			}else if (pI2Cx == I2C2)
			{
				I2C2_PCLK_DI();
			}else if (pI2Cx == I2C3)
			{
				I2C3_PCLK_DI();
			}
		}
}

/*
 * Get the Flag Status of the SR1 Register
 */

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx,uint8_t FlagName)
{
	if(pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*
 * Peripheral setup
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}
	else if(EnorDi == DISABLE)
	{
		pI2Cx->CR1 |= ~(1 << I2C_CR1_PE);
	}
}
void I2C_AckControl(I2C_RegDef_t *I2Cx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		I2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}
	else
	{
		I2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}

/*
 * Peripheral Init
 */

void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0;

	//0.Enable the Clock for I2Cx Init
	I2C_PeriClockControl(pI2CHandle->I2Cx, ENABLE);

	//1. Program the FREQ field in CR2 Register
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value()/1000000U;
	pI2CHandle->I2Cx->CR2 = (tempreg & (0x3F));

	//2. Program the OAR1 Register (Own Address Register)
	tempreg = 0;
	tempreg |= (pI2CHandle->I2C_Config.I2C_DeviceAddress << 1);
	tempreg |= (1 << 14);
	pI2CHandle->I2Cx->OAR1 = tempreg;

	//3. program the CCR field in CCR Register
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SclSpeed <= I2C_SCL_SPEED_SM)
	{
		//Standard Mode
		ccr_value = (RCC_GetPCLK1Value())/(2 * pI2CHandle->I2C_Config.I2C_SclSpeed);
		tempreg |= ccr_value & (0xFFF);
	}
	else
	{
		//Fast Mode
		tempreg |= (1 << I2C_CCR_FS);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << I2C_CCR_DUTY);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = (RCC_GetPCLK1Value())/(3 * pI2CHandle->I2C_Config.I2C_SclSpeed);
		}
		else if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_16_9)
		{
			ccr_value = (RCC_GetPCLK1Value())/(25 * pI2CHandle->I2C_Config.I2C_SclSpeed);
		}
		tempreg |= ccr_value & (0xFFF);
	}
	pI2CHandle->I2Cx->CCR = tempreg;

	//4.Configure the TRISE Register

	if(pI2CHandle->I2C_Config.I2C_SclSpeed <= I2C_SCL_SPEED_SM)
	{
		tempreg = ((RCC_GetPCLK1Value()) / 1000000U) + 1;
	}
	else
	{
		tempreg = (((RCC_GetPCLK1Value())*300)/1000000000U) + 1;
	}

	//5.Configure the TRISE register after calculating the TRISE value.
	pI2CHandle->I2Cx->TRISE = (tempreg & 0x3F);
}

/*
 * Peripheral Register Reset
 */

void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if(pI2Cx == I2C1)
	{
		I2C1_REG_RESET();
	}else if (pI2Cx == I2C2)
	{
		I2C2_REG_RESET();
	}else if (pI2Cx == I2C3)
	{
		I2C3_REG_RESET();
	}
}

/*
 * Master send Data
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer,uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
{
	uint32_t dummyread;
	//1.Generate the start condition
	pI2CHandle->I2Cx->CR1 |= (1 << I2C_CR1_START);

	//2.Read the Status flag of the SR1 register to Confirm that start generation is completed.
	while(! I2C_GetFlagStatus(pI2CHandle->I2Cx,I2C_FLAG_SB));

	//3.Send the address of the slave with r/m = 0 (total 8 Bits)
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1 << 0); // Slave Address + r/w Bit = 0
	pI2CHandle->I2Cx->DR = SlaveAddr;

	//4.Confirm the Address phase is completed by checking the ADDR field in SR1 register
	while(!I2C_GetFlagStatus(pI2CHandle->I2Cx, I2C_FLAG_ADDR));

	//5.Clear Address Flag by  reading SR1 register followed reading SR2 register
	//Note : Until ADDR is cleared SCL will be stretched to Low
	dummyread = pI2CHandle->I2Cx->SR1;
	dummyread = pI2CHandle->I2Cx->SR2;
	(void)dummyread;

	//6.Send the data Until length becomes zero
	while(Len > 0)
	{
		while(!I2C_GetFlagStatus(pI2CHandle->I2Cx, I2C_FLAG_TXE)); // Wait till TXE is set
		pI2CHandle->I2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}

	//7. When Len = 0 wait until TXE and BTF field is set before generating the STOP Condition
	while(!I2C_GetFlagStatus(pI2CHandle->I2Cx, I2C_FLAG_TXE));

	while(!I2C_GetFlagStatus(pI2CHandle->I2Cx, I2C_FLAG_BTF));

	//8.Generate the STOP Condition
	if(Sr == I2C_NO_SR)
	{
	pI2CHandle->I2Cx->CR1 |= (1 << I2C_CR1_STOP);
	}
}

/*
 * Master Receive Data
 */

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer,uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
{
	uint32_t dummyread;
	//1.Generate the start condition
	pI2CHandle->I2Cx->CR1 |= (1 << I2C_CR1_START);

	//2.Read the Status flag of the SR1 register to Confirm that start generation is completed
	while(! I2C_GetFlagStatus(pI2CHandle->I2Cx,I2C_FLAG_SB));

	//3.Send the address of the slave with r/m = 1 (total 8 Bits)
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= (1 << 0); // Slave Address + r/w Bit = 1 (Reading) from Slave
	pI2CHandle->I2Cx->DR = SlaveAddr;

	//4.Confirm the Address phase is completed by checking the ADDR field in SR1 register
	while(!I2C_GetFlagStatus(pI2CHandle->I2Cx, I2C_FLAG_ADDR));

	//Procedure to Read Only 1 Byte of data from slave
	if(Len == 1)
	{
		//Disable Acking
		pI2CHandle->I2Cx->CR1 &= ~( 1 << I2C_CR1_ACK);

		//Clear the ADDR Flag
		dummyread = pI2CHandle->I2Cx->SR1;
		dummyread = pI2CHandle->I2Cx->SR2;
		(void)dummyread;

		//Wait Until RXNE flag becomes 1
		while(!(I2C_GetFlagStatus(pI2CHandle->I2Cx, I2C_FLAG_RXNE)));

		//Generate STOP Condition
		if(Sr == I2C_NO_SR)
		{
		pI2CHandle->I2Cx->CR1 |= (1 << I2C_CR1_STOP);
		}

		//Read data in to the Buffer
		*pRxBuffer = pI2CHandle->I2Cx->DR;
	}

	//Procedure to Read if Data Length is Greater than 1
	if(Len > 1)
	{
		//Clear the ADDR flag
		dummyread = pI2CHandle->I2Cx->SR1;
		dummyread = pI2CHandle->I2Cx->SR2;
		(void)dummyread;

		//Read the data until the length becomes Zero
		for(uint32_t i = Len;i > 0; i-- )
		{
			//Wait until RXNE Flag Becomes 1
			while(!(I2C_GetFlagStatus(pI2CHandle->I2Cx, I2C_FLAG_RXNE)));

			if(i == 2)  // Last two Bytes are Remaining
			{
				//Clear the ACK Bit
				pI2CHandle->I2Cx->CR1 &= ~(1 << I2C_CR1_ACK);

				//Generate the stop Bit
				if(Sr == I2C_NO_SR)
				{
				pI2CHandle->I2Cx->CR1 |= (1 << I2C_CR1_STOP);
				}
			}

			//Read the data from the register to buffer
			*pRxBuffer = pI2CHandle->I2Cx->DR;

			//Increment the buffer
			pRxBuffer++;
		}
	}
	//Re-Enable the Acking
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
	pI2CHandle->I2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}
}

/*
 * Slave Send and Receive Data
 */
void I2C_SlaveSendData(I2C_RegDef_t *I2Cx,uint8_t data)
{
	I2Cx->DR = data;
}
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *I2Cx)
{
	return (uint8_t)I2Cx->DR;
}

/*********************************************************************
 * @fn      		  - I2C_MasterSendDataIT
 */
uint8_t  I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DeviceAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		pI2CHandle->I2Cx->CR1 |= (1 << I2C_CR1_START);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->I2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->I2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->I2Cx->CR2 |= (1 << I2C_CR2_ITERREN);

	}

	return busystate;

}
/*********************************************************************
 * @fn      		  - I2C_MasterReceiveDataIT
 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DeviceAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		pI2CHandle->I2Cx->CR1 |= (1 << I2C_CR1_START);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->I2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->I2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->I2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}

	return busystate;
}
/*
 * Interrupt Service Routine(ISR)
 */
/*********************************************************************
 * @fn      		  - I2C_EV_IRQHandling */


void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	//Interrupt handling for both master and slave mode of a device
	uint32_t temp1,temp2,temp3,dummyread;

	temp1 = pI2CHandle->I2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->I2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);

	temp3 = pI2CHandle->I2Cx->SR1 & (1 << I2C_SR1_SB);
	//1. Handle For interrupt generated by SB event
	//	Note : SB flag is only applicable in Master mode
	if(temp1 && temp3)
	{
		//SB Flag is Set,Execute Address Phase, Its only valid  for Master Mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			//Send the address of the slave with r/m = 0 (total 8 Bits)
			pI2CHandle->DeviceAddr = pI2CHandle->DeviceAddr << 1;
			pI2CHandle->DeviceAddr &= ~(1 << 0); // Slave Address + r/w Bit = 0
			pI2CHandle->I2Cx->DR = pI2CHandle->DeviceAddr;
		}
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			//Read the address of the slave with r/m = 1 (total 8 Bits)
			pI2CHandle->DeviceAddr = pI2CHandle->DeviceAddr << 1;
			pI2CHandle->DeviceAddr |= (1 << 0); // Slave Address + r/w Bit = 1 (Reading) from Slave
			pI2CHandle->I2Cx->DR = pI2CHandle->DeviceAddr;
		}
	}

	temp3 = pI2CHandle->I2Cx->SR1 & (1 << I2C_SR1_ADDR);
	//2. Handle For interrupt generated by ADDR event
	//Note : When master mode : Address is sent
	//		 When Slave mode   : Address matched with own address
	if(temp1 && temp3)
	{
		//ADDR Flag is set
		dummyread = pI2CHandle->I2Cx->SR1;
		dummyread = pI2CHandle->I2Cx->SR2;
		(void)dummyread;
	}

	temp3 = pI2CHandle->I2Cx->SR1 & (1 << I2C_SR1_BTF);
	//3. Handle For interrupt generated by BTF(Byte Transfer Finished) event
	if(temp1 && temp3)
	{
		//BTF Flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			//Make sure TXE is also Set
			if(pI2CHandle->I2Cx->SR1 & (1 << I2C_SR1_TXE))
			{
				//BTF and TXE is 1
				if(pI2CHandle->TxLen == 0)
				{
				//1.Generate Stop Condition
				if(pI2CHandle->Sr == I2C_NO_SR)
				{
				pI2CHandle->I2Cx->CR1 |= (1 << I2C_CR1_STOP);
				}

				//2.Reset all member elements and handle structure
				I2C_CloseSendData(pI2CHandle);

				//3.Notify the application about the transmission Complete
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}
		}
	}

	temp3 = pI2CHandle->I2Cx->SR1 & (1 << I2C_SR1_STOPF);
	//4. Handle For interrupt generated by STOPF event
	// Note : Stop detection flag is applicable only slave mode . For master this flag will never be set
	if(temp1 && temp3)
	{
		//STOPF Flag is set,Clear the STOPF use procedure from reference manual
		pI2CHandle->I2Cx->CR1 |= 0x0000;

		//Notify the application about STOP is Detected
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}

	temp3 = pI2CHandle->I2Cx->SR1 & (1 << I2C_SR1_TXE);
	//5. Handle For interrupt generated by TXE event
	if(temp2 && temp1 && temp3)
	{
		//TXE Flag Set
		//Check for the Device Mode
		if(pI2CHandle->I2Cx->SR2 & (1 << I2C_SR2_MSL))
		{
			//Check for the TXE state
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				if(pI2CHandle->TxLen > 0)
				{
					//1.Load the DR
					pI2CHandle->I2Cx->DR = *(pI2CHandle->pTxBuffer);

					//2.Increment the Buffer
					pI2CHandle->pTxBuffer++;

					//3.Decrement the TX length
					pI2CHandle->TxLen--;
				}
			}
		}
		else
		{
			//Check for TRA status, and also device is in transmitter mode
			if(pI2CHandle->I2Cx->SR2 & (1 << I2C_SR2_TRA))
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
			}
		}

	}

	temp3 = pI2CHandle->I2Cx->SR1 & (1 << I2C_SR1_RXNE);
	//6. Handle For interrupt generated by RXNE event
	if(temp2 && temp1 && temp3)
	{
		//Check for Device Mode
		if(pI2CHandle->I2Cx->SR2 & (1 << I2C_SR2_MSL))
		{
			//RXNE Flag Set
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				//We Have to do Data Reception
				if(pI2CHandle->RxSize == 1)
				{
					I2C_ClearAddrFlag(pI2CHandle);
					//Read from DR to RxBuffer
					*pI2CHandle->pRxBuffer = pI2CHandle->I2Cx->DR;

					//Decrement the length
					pI2CHandle->RxSize--;
				}
				if(pI2CHandle->RxSize > 1)
				{
					if(pI2CHandle->RxSize == 2)
					{
						//Clear the ACK Bit
						pI2CHandle->I2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
					}

					//Read the data from the register to buffer
					*pI2CHandle->pRxBuffer = pI2CHandle->I2Cx->DR;

					//Decrement the RxSize
					pI2CHandle->RxSize--;

					//Increment the buffer
					pI2CHandle->pRxBuffer++;
				}
				if(pI2CHandle->RxSize == 0)
				{
					//1.Generate the STOP Condition
					if(pI2CHandle->Sr == I2C_NO_SR)
					{
						pI2CHandle->I2Cx->CR1 |= (1 << I2C_CR1_STOP);
					}

					//2.Close the RX
					I2C_CloseReciveData(pI2CHandle);

					//3. Notify the Application Call back
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);

				}
			}
		}
		else
		{
			//Device is in Receiver Mode
			if(!(pI2CHandle->I2Cx->SR2 & (1 << I2C_SR2_TRA)))
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
			}
		}
	}
}

/*********************************************************************
 * @fn      		  - I2C_ER_IRQHandling */

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->I2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->I2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->I2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->I2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->I2Cx->SR1 &= ~(1 << I2C_SR1_ARLO);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);

	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->I2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag
		pI2CHandle->I2Cx->SR1 &= ~(1 << I2C_SR1_AF);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->I2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->I2Cx->SR1 &= ~(1 << I2C_SR1_OVR);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->I2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		pI2CHandle->I2Cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
	}

}
void I2C_SlaveEnableDisableCallbackEventd(I2C_Handle_t *pI2CHandle,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pI2CHandle->I2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
		pI2CHandle->I2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
		pI2CHandle->I2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
	}
	else if(EnorDi == DISABLE)
	{
		pI2CHandle->I2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
		pI2CHandle->I2Cx->CR2 &= ~(1 << I2C_CR2_ITERREN);
		pI2CHandle->I2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);
	}
}

/*
 * IRQ Configuration and ISR Handling
 */

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );
}


void I2C_CloseReciveData(I2C_Handle_t *pI2CHandle)
{
	//Disable all the Interrupt
	pI2CHandle->I2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	pI2CHandle->I2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	//Reset all member states
	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->RxSize = 0;
	pI2CHandle->RxLen = 0;
	pI2CHandle->pRxBuffer = NULL;
	//Enable the Acking
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		pI2CHandle->I2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}
}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	//Disable all the Interrupt
	pI2CHandle->I2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	pI2CHandle->I2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	//Reset all member states
	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->TxLen = 0;
	pI2CHandle->pTxBuffer = NULL;
}
/*
 * Clear ADDR Flag
 */
static void I2C_ClearAddrFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummyread;
	//Check for Device Mode
	if(pI2CHandle->I2Cx->SR2 & (1 << I2C_SR2_MSL))
	{
		//Device is in Master mode, now check for Busy in TX..?
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxSize == 1)
			{
				//Manage Acking
				pI2CHandle->I2Cx->CR1 &= ~( 1 << I2C_CR1_ACK);

				//Clear the ADDR Flag
				dummyread = pI2CHandle->I2Cx->SR1;
				dummyread = pI2CHandle->I2Cx->SR2;
				(void)dummyread;

			}
		}
		else
		{
			//Clear the ADDR Flag
			dummyread = pI2CHandle->I2Cx->SR1;
			dummyread = pI2CHandle->I2Cx->SR2;
			(void)dummyread;
		}
	}
	else
	{
		//Clear the ADDR Flag
		dummyread = pI2CHandle->I2Cx->SR1;
		dummyread = pI2CHandle->I2Cx->SR2;
		(void)dummyread;
	}
}

/*
 * Application Event Call back
 */

__weak void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv)
{

}
