/*
 * stm32f4xx_spi_driver.c
 *
 *  Created on: Jan 26, 2020
 *      Author: nemanja
 */
#include <stm32f446xx_spi_driver.h>


/*
 * Helper functions
 */
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);


/*
 * Peripheral Clock setup
 */

/*****************************************************************
 * @fn				- SPI_PeriClockControl
 *
 * @brief			- This function enables or disables peripheral
 * 					  clock for the given SPI port
 *
 * @param[in]		- Base address of the SPI peripheral
 * @param[in]		- Macros:	Enable or Disable
 *
 * @return			- None
 *
 * @Note			- None
 *
 *****************************************************************/
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
		else if(pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}
	}
	else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
		else if(pSPIx == SPI4)
		{
			SPI4_PCLK_DI();
		}
	}
}

/*
 * Init and De-Init
 */
/*****************************************************************
 * @fn				- SPI_Init
 *
 * @brief			- This function initialize SPI peripherals
 *
 * @param[in]		- Pointer to SPI Handle structure
 *
 * @return			- None
 *
 * @Note			- None
 *
 *****************************************************************/
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	/* SPI_CR1 register configuration */
	uint32_t tempreg = 0;

	/* Enable peripheral clock */
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	/* Device mode configuration */
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	/* Bus configuration */
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		/* Bidirectional mode clear */
		tempreg &= ~(1 << SPI_CR1_BIDI_MODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		/* Bidirectional mode set */
		tempreg |= (1 << SPI_CR1_BIDI_MODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RX_ONLY)
	{
		/* Bidirectional mode clear */
		tempreg &= ~(1 << SPI_CR1_BIDI_MODE);

		/* Rx only mode set */
		tempreg |= (1 << SPI_CR1_RX_ONLY);
	}

	/* SPI serial clock speed (baud rate) configuration */
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	/* DFF configuration */
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	/* CPOL configuration */
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	/* CPHA configuration */
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	/* Save temperg in CR1 register */
	pSPIHandle->pSPIx->CR1 = tempreg;

}


/*****************************************************************
 * @fn				- SPI_DeInit
 *
 * @brief			- This function de-initialize SPI peripherals
 *
 * @param[in]		- Base address of the SPI peripheral
 *
 * @return			- None
 *
 * @Note			- None
 *
 *****************************************************************/
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}
	else if(pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}
	else if(pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
	else if(pSPIx == SPI4)
	{
		SPI4_REG_RESET();
	}
}


/*****************************************************************
 * @fn				- SPI_GetFlagStatus
 *
 * @brief			- This function returns if bit in register is
 * 					  set or not
 *
 * @param[in]		- Base address of the SPI peripheral
 * @param[in]		- Name of flag
 *
 * @return			- Flag status (True/False)
 *
 * @Note			- None
 *
 *****************************************************************/
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*
 * Data send and receive
 */
/*****************************************************************
 * @fn				- SPI_SendData
 *
 * @brief			- This function sends data over SPI peripheral
 *
 * @param[in]		- Base address of the SPI peripheral
 * @param[in]		- Transmit buffer
 * @param[in]		- Length of transmit buffer
 *
 * @return			- None
 *
 * @Note			- None
 *
 *****************************************************************/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Length)
{
	while(Length > 0)
	{
		/* Wait until TXE is set */
		while(SPI_GetFlagStatus(pSPIx->SR, SPI_FLAG_TXE) == (uint8_t)FLAG_RESET);

		if( pSPIx->CR1 & (1 << SPI_CR1_DFF) )
		{
			/* Load data into data register */
			/* 16 bit */
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Length--;
			Length--;
			(uint16_t*)pTxBuffer++;
		}
		else
		{
			/* 8 bit */
			pSPIx->DR = *pTxBuffer;
			Length--;
			pTxBuffer++;
		}
	}
}


/*****************************************************************
 * @fn				- SPI_ReceivedData
 *
 * @brief			- This function receives data over SPI
 * 					  peripheral
 *
 * @param[in]		- Base address of the SPI peripheral
 * @param[in]		- Transmit buffer
 * @param[in]		- Length of transmit buffer
 *
 * @return			- None
 *
 * @Note			- None
 *
 *****************************************************************/
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Length)
{
	while(Length > 0)
	{
		/* Wait until RXNE is set */
		while(SPI_GetFlagStatus(pSPIx->SR, SPI_FLAG_RXNE) == (uint8_t)FLAG_RESET);

		if( pSPIx->CR1 & (1 << SPI_CR1_DFF) )
		{
			/* Load data from DR to RxBuffer */
			/* 16 bit */
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			Length--;
			Length--;
			(uint16_t*)pRxBuffer++;
		}
		else
		{
			/* 8 bit */
			*(pRxBuffer) = pSPIx->DR;
			Length--;
			pRxBuffer++;
		}
	}
}


/*****************************************************************
 * @fn				- SPI_SendDataInterruptMode
 *
 * @brief			- This function sends data over SPI
 * 					  peripheral in Interrupt mode
 *
 * @param[in]		- Pointer to SPI Handle structure
 * @param[in]		- Transmit buffer
 * @param[in]		- Length of transmit buffer
 *
 * @return			- Tx State
 *
 * @Note			- None
 *
 *****************************************************************/
uint8_t SPI_SendDataInterruptMode(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Length)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
		/* Save Tx buffer address and length information in global variables */
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Length;

		/* Mark SPI state as busy so that no other code can take over SPI peripheral until transmission is over */
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		/* Enable TXEIE control bit to get interrupt whenever TXE flag is set in SR */
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}
	/* DBG->Data transmission*/
	return state;
}


/*****************************************************************
 * @fn				- SPI_ReceiveDataInterruptMode
 *
 * @brief			- This function receives data over SPI
 * 					  peripheral in Interrupt mode
 *
 * @param[in]		- Pointer to SPI Handle structure
 * @param[in]		- Transmit buffer
 * @param[in]		- Length of transmit buffer
 *
 * @return			- Rx State
 *
 * @Note			- None
 *
 *****************************************************************/
uint8_t SPI_ReceiveDataInterruptMode(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Length)
{
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX)
	{
		/* Save Rx buffer address and length information in global variables */
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Length;

		/* Mark SPI state as busy so that no other code can take over SPI peripheral until transmission is over */
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		/* Enable RXNEIE control bit to get interrupt whenever RXE flag is set in SR */
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
	}

	return state;
}


/*
 * IRQ Configuration and ISR handling
 */
/*****************************************************************
 * @fn				- SPI_IRQConfig
 *
 * @brief			- This function configures interrupt
 *
 * @param[in]		- IRQ Interrupt number
 * @param[in]		- Macro: Enable/Disable
 *
 * @return			- None
 *
 * @Note			- None
 *
 *****************************************************************/
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			/* Program ISER0 register */
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			/* Program ISER1 register (32 to 63) */
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			/* Program ISER2 register (64 to 95) */
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			/* Program ICER0 register */
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			/* Program ICER1 register (32 to 63) */
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			/* Program ICER2 register (64 to 95) */
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
}


/*****************************************************************
 * @fn				- SPI_IRQPriorityConfig
 *
 * @brief			- This function configures interrupt priority
 *
 * @param[in]		- IRQ Interrupt number
 * @param[in]		- IRQ interrupt priority
 *
 * @return			- None
 *
 * @Note			- None
 *
 *****************************************************************/
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}


/*****************************************************************
 * @fn				- SPI_IRQHandling
 *
 * @brief			- This function handle interrupts
 *
 * @param[in]		- Handle structure
 *
 * @return			- None
 *
 * @Note			- None
 *
 *****************************************************************/
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1;
	uint8_t temp2;

	/* Check for TXE */
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if(temp1 && temp2)
	{
		/* Handle TXE */
		spi_txe_interrupt_handle(pHandle);
	}

	/* Check for RXNE */
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if(temp1 && temp2)
	{
		/* Handle RXNE */
		spi_rxne_interrupt_handle(pHandle);
	}

	/* Check for OVR flag */
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if(temp1 && temp2)
	{
		/* Handle OVR Error */
		spi_ovr_err_interrupt_handle(pHandle);
	}

}


/*****************************************************************
 * @fn				- SPI_PeripheralControl
 *
 * @brief			- This function sets SPI peripheral control
 *
 * @param[in]		- Base address of the SPI peripheral
 * @param[in]		- Enable or Disable command
 *
 * @return			- None
 *
 * @Note			- None
 *
 *****************************************************************/
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
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


/*****************************************************************
 * @fn				- SPI_SSIConfig
 *
 * @brief			- This function sets SSI register
 *
 * @param[in]		- Base address of the SPI peripheral
 * @param[in]		- Enable or Disable command
 *
 * @return			- None
 *
 * @Note			- None
 *
 *****************************************************************/
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
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


/*****************************************************************
 * @fn				- SPI_SSOEConfig
 *
 * @brief			- This function sets SSEO register
 *
 * @param[in]		- Base address of the SPI peripheral
 * @param[in]		- Enable or Disable command
 *
 * @return			- None
 *
 * @Note			- None
 *
 *****************************************************************/
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}
	else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}


/*****************************************************************
 * @fn				- SPI_CloseTransmission
 *
 * @brief			- This function close SPI transmission
 *
 * @param[in]		- Handle structure
 *
 * @return			- None
 *
 * @Note			- None
 *
 *****************************************************************/
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}


/*****************************************************************
 * @fn				- SPI_CloseReception
 *
 * @brief			- This function close SPI reception
 *
 * @param[in]		- Handle structure
 *
 * @return			- None
 *
 * @Note			- None
 *
 *****************************************************************/
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}


/*****************************************************************
 * @fn				- SPI_ClearOVRFlag
 *
 * @brief			- This function clears OVR flag
 *
 * @param[in]		- Base address of the SPI peripheral
 *
 * @return			- None
 *
 * @Note			- None
 *
 *****************************************************************/
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}


/*****************************************************************
 *               Helper functions implementation                 *
 *****************************************************************/


/*****************************************************************
 * @fn				- SPI_ApplicationEventCallback
 *
 * @brief			- Application event callback function
 *
 * @param[in]		- Handle structure
 * @param[in]		- Application event
 *
 * @return			- None
 *
 * @Note			- None
 *
 *****************************************************************/
__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent)
{
	/* This is a week implementation. The application may override this function. */
}


/*****************************************************************
 * @fn				- spi_txe_interrupt_handle
 *
 * @brief			- This function handles TXE in interrupt mode
 *
 * @param[in]		- Pointer to SPI Handle structure
 *
 * @return			- None
 *
 * @Note			- None
 *
 *****************************************************************/
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if( pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF) )
	{
		/* Load data into data register */
		/* 16 bit */
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}
	else
	{
		/* 8 bit */
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if(!pSPIHandle->TxLen)
	{
		/* Tx is zero. Close SPI communication and inform application about it.
		 * Prevents interrupts from setting up of TXE flag. */
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}


/*****************************************************************
 * @fn				- spi_rxne_interrupt_handle
 *
 * @brief			- This function handles RXNE in interrupt mode
 *
 * @param[in]		- Pointer to SPI Handle structure
 *
 * @return			- None
 *
 * @Note			- None
 *
 *****************************************************************/
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if( pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF) )
	{
		/* Load data from data register into buffer */
		/* 16 bit */
		*((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;
		(uint16_t*)pSPIHandle->pRxBuffer++;
	}
	else
	{
		/* 8 bit */
		*pSPIHandle->pRxBuffer = pSPIHandle->pSPIx->DR;//DBG->Check brackets
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}

	if(!pSPIHandle->RxLen)
	{
		/* Rx is zero. Close SPI communication and inform application about it.
		 * Prevents interrupts from setting up of RXNE flag. */
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}


/*****************************************************************
 * @fn				- spi_ovr_err_interrupt_handle
 *
 * @brief			- This function handles OVR_ERR in
 * 					  interrupt mode
 *
 * @param[in]		- Pointer to SPI Handle structure
 *
 * @return			- None
 *
 * @Note			- None
 *
 *****************************************************************/
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;

	/* Clear OVR flag */
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;

	/* Inform application */
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

