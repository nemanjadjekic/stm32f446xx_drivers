/*
 * stm32f4xx_spi_driver.c
 *
 *  Created on: Jan 26, 2020
 *      Author: nemanja
 */

#include "stm32f446xx_spi_driver.h"

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
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << 2;

	/* Bus configuration */
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		/* Bidirectional mode clear */
		tempreg &= ~(1 << 15);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		/* Bidirectional mode set */
		tempreg |= (1 << 15);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RX_ONLY)
	{
		/* Bidirectional mode clear */
		tempreg &= ~(1 << 15);

		/* Rx only mode set */
		tempreg &= ~(1 << 10);
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
		while(SPI_GetFlagStatus(pSPIx->SR, SPI_TXE_FLAG) == FLAG_RESET);

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
		while(SPI_GetFlagStatus(pSPIx->SR, SPI_RXNE_FLAG) == FLAG_RESET);

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
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			Length--;
			pRxBuffer++;
		}
	}
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
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);


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
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);


/*****************************************************************
 * @fn				- GPIO_IRQHandling
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
void SPI_IRQHandling(SPI_Handle_t *pHandle);


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
