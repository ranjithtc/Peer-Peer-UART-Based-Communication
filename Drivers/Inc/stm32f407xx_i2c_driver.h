/*
 * stm32f407xx_i2c_driver.h
 *
 *  Created on: Dec 29, 2022
 *      Author: ranji
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

#include <stdint.h>
#include "stm32f407xx.h"

/*
 * Configuration Structure for I2Cx peripheral.
 */

typedef struct
{
	uint32_t		I2C_SclSpeed;
	uint8_t			I2C_DeviceAddress;
	uint8_t			I2C_ACKControl;
	uint16_t		I2C_FMDutyCycle;
}I2C_Config_t;

/*
 * Handle Structure for I2Cx peripheral.
 */

typedef struct
{
	I2C_RegDef_t 	*I2Cx;
	I2C_Config_t	I2C_Config;
	uint8_t			*pTxBuffer;
	uint8_t			*pRxBuffer;
	uint32_t		TxLen;
	uint32_t		RxLen;
	uint8_t			TxRxState;
	uint8_t			DeviceAddr;
	uint32_t		RxSize;
	uint8_t			Sr;
}I2C_Handle_t;

/*
 * Application states
 */
#define I2C_READY   		0
#define I2C_BUSY_IN_RX 		1
#define I2C_BUSY_IN_TX 		2

/*
 * I2C Application state Events
 */
#define I2C_EV_TX_CMPLT		0
#define I2C_EV_RX_CMPLT		1
#define I2C_EV_STOP			2
#define I2C_ERROR_BERR		3
#define I2C_ERROR_ARLO		4
#define I2C_ERROR_AF		5
#define I2C_ERROR_OVR		6
#define I2C_ERROR_TIMEOUT	7
#define I2C_EV_DATA_REQ		8
#define I2C_EV_DATA_RCV		9

/*
 * I2C Application error states
 */
#define I2C_ERROR_BERR  3
#define I2C_ERROR_ARLO  4
#define I2C_ERROR_AF    5
#define I2C_ERROR_OVR   6
#define I2C_ERROR_TIMEOUT 7

/*
 * Possible values for @I2C_SclSpeed
 */
#define I2C_SCL_SPEED_SM		100000
#define I2C_SCL_SPEED_FM_2K		200000
#define I2C_SCL_SPEED_FM_4K		400000

/*
 * Possible Values for @I2C_ACKControl
 */
#define I2C_ACK_ENABLE		1
#define I2C_ACK_DISABLE		0

/*
 * Possible Values for @I2C_FMDutyCycle
 */

#define I2C_FM_DUTY_2		0
#define I2C_FM_DUTY_16_9	1

/*
 * Some flags used in I2C
 */

#define I2C_FLAG_SB			(1 << I2C_SR1_SB)
#define I2C_FLAG_BTF		(1 << I2C_SR1_BTF)
#define I2C_FLAG_STOPF		(1 << I2C_SR1_STOPF)
#define I2C_FLAG_RXNE		(1 << I2C_SR1_RXNE)
#define I2C_FLAG_TXE		(1 << I2C_SR1_TXE)
#define I2C_FLAG_BERR		(1 << I2C_SR1_BERR)
#define I2C_FLAG_ARLO		(1 << I2C_SR1_ARLO)
#define I2C_FLAG_AF			(1 << I2C_SR1_AF)
#define I2C_FLAG_OVR		(1 << I2C_SR1_OVR)
#define I2C_FLAG_TIMEOUT 	(1 << I2C_SR1_TIMEOUT)
#define I2C_FLAG_ADD10		(1 << I2C_SR1_ADD10)
#define I2C_FLAG_ADDR		(1 << I2C_SR1_ADDR)

#define I2C_NO_SR	RESET
#define I2C_SR		SET
/*
 * Peripheral Clock setup
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
 * Peripheral setup
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
/*
 * To get the status of the flag
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx,uint8_t FlagName);
void I2C_CloseReciveData(I2C_Handle_t *pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);

/*
 * Init and DeInit
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*
 * Master Send Data
 */
void I2C_AckControl(I2C_RegDef_t *I2Cx,uint8_t EnorDi);
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer,uint32_t Len,uint8_t SlaveAddr,uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer,uint32_t Len,uint8_t SlaveAddr,uint8_t Sr);
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer,uint32_t Len,uint8_t SlaveAddr,uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer,uint32_t Len,uint8_t SlaveAddr,uint8_t Sr);
void I2C_SlaveSendData(I2C_RegDef_t *I2Cx,uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *I2Cx);

/*
 * Interrupt Send and Receive data.
 */
uint8_t I2C_SendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len);
uint8_t I2C_ReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t Len);
void I2C_SlaveEnableDisableCallbackEventd(I2C_Handle_t *pI2CHandle,uint8_t EnorDi);
/*
 * IRQ Configuration and ISR Handling
 */

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);
/*
 * Application Event Call back
 */

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv);

/*
 *I2C SSI Configuration
 */
void I2C_SSIConfig(I2C_RegDef_t *pI2Cx,uint8_t EnorDi);

#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
