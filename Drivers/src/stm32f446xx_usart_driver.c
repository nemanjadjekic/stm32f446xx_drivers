/*
 * stm32f446xx_usart_driver.c
 *
 *  Created on: Feb 29, 2020
 *      Author: nemanja
 */

#include <stm32f446xx_usart_driver.h>

/*
 * Peripheral Clock setup
 */
/*****************************************************************
 * @fn			- USART_PeriClockControl
 *
 * @brief		- This function enables or disables peripheral
 * 			  clock for the given U(S)ART port
 *
 * @param[in]		- Base address of the U(S)ART peripheral
 * @param[in]		- Macros: Enable or Disable
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************/
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_EN();
		}
		else if(pUSARTx == USART2)
		{
			USART2_PCLK_EN();
		}
		else if(pUSARTx == USART3)
		{
			USART3_PCLK_EN();
		}
		else if(pUSARTx == UART4)
		{
			UART4_PCLK_EN();
		}
		else if(pUSARTx == UART5)
		{
			UART5_PCLK_EN();
		}
		else if(pUSARTx == USART6)
		{
			USART6_PCLK_EN();
		}
	}
	else
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_DI();
		}
		else if(pUSARTx == USART2)
		{
			USART2_PCLK_DI();
		}
		else if(pUSARTx == USART3)
		{
			USART3_PCLK_DI();
		}
		else if(pUSARTx == UART4)
		{
			UART4_PCLK_DI();
		}
		else if(pUSARTx == UART5)
		{
			UART5_PCLK_DI();
		}
		else if(pUSARTx == USART6)
		{
			USART6_PCLK_DI();
		}
	}
}


/*****************************************************************
 * @fn			- USART_PeripheralControl
 *
 * @brief		- This function sets U(S)ART peripheral control
 *
 * @param[in]		- Base address of the U(S)ART peripheral
 * @param[in]		- Enable or Disable command
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************/
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pUSARTx->CR1 |= (1 << USART_CR1_UE);
	}
	else
	{
		pUSARTx->CR1 &= ~(1 << USART_CR1_UE);
	}
}


/*
 * Init and De-Init
 */
/*****************************************************************
 * @fn			- USART_Init
 *
 * @brief		- This function initialize USART peripherals
 *
 * @param[in]		- Pointer to USART Handle structure
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************/
void USART_Init(USART_Handle_t *pUSARTHandle)
{
	uint32_t tempreg = 0;

	/* CR1 Configuration */
	/* Enabling clock for U(S)ART peripheral */
	USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

	/* Enabling USART Tx and Rx engines according to the USART Mode Config. item */
	if ( pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		/* Enabling Receiver bit field */
		tempreg |= (1 << USART_CR1_RE);
	}
	else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		/* Enabling Transmitter bit field */
		tempreg |= (1 << USART_CR1_TE);
	}
	else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		/* Enabling both, Transmitter and Receiver bit fields */
		tempreg |= (1 << USART_CR1_TE);
		tempreg |= (1 << USART_CR1_RE);
	}

	/* Configuring Word Length configuration item */
	tempreg |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M;

	/* Parity control bit fields control */
	if ( pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		/* Enabling parity control. EVEN parity select by default */
		tempreg |= (1 << USART_CR1_PCE);

	}else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD )
	{
		/* Enabling parity control */
	    tempreg |= (1 << USART_CR1_PCE);

	    /* Enabling ODD parity */
	    tempreg |= (1 << USART_CR1_PS);
	}

	/* Programming CR1 register */
	pUSARTHandle->pUSARTx->CR1 = tempreg;


	/* CR2 Configuration */
	tempreg=0;

	/* Configuring number of STOP bits inserted during USART frame transmission */
	tempreg |= pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP;

	/* Programming CR2 register */
	pUSARTHandle->pUSARTx->CR2 = tempreg;


	/* CR3 Configuration */
	tempreg=0;

	/* USART Hardware Flow control Configuration */
	if ( pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		/* Enabling CTS Flow control */
		tempreg |= (1 << USART_CR3_CTSE);
	}
	else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		/* Enabling RTS flow control */
		tempreg |= (1 << USART_CR3_RTSE);

	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		/* Enabling both CTS and RTS Flow control */
		tempreg |= (1 << USART_CR3_CTSE);
	    tempreg |= (1 << USART_CR3_RTSE);
	}

	pUSARTHandle->pUSARTx->CR3 = tempreg;

	/* Configuring Baudrate */
	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);
}


/*****************************************************************
 * @fn			- USART_DeInit
 *
 * @brief		- This function de-initialize U(S)ART peripherals
 *
 * @param[in]		- Base address of the U(S)ART peripheral
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************/
void USART_DeInit(USART_RegDef_t *pUSARTx)
{
	if(pUSARTx == USART1)
	{
		USART1_REG_RESET();
	}
	else if(pUSARTx == USART2)
	{
		USART2_REG_RESET();
	}
	else if(pUSARTx == USART3)
	{
		USART3_REG_RESET();
	}
	else if(pUSARTx == UART4)
	{
		UART4_REG_RESET();
	}
	else if(pUSARTx == UART5)
	{
		UART5_REG_RESET();
	}
	else if(pUSARTx == USART6)
	{
		USART6_REG_RESET();
	}
}


/*****************************************************************
 * @fn			- USART_GetFlagStatus
 *
 * @brief		- This function returns if bit in register is
 * 			  set or not
 *
 * @param[in]		- Base address of the U(S)ART peripheral
 * @param[in]		- Name of flag
 *
 * @return		- Flag status (True/False)
 *
 * @Note		- None
 *
 *****************************************************************/
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName)
{
	if(pUSARTx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}


/*****************************************************************
 * @fn			- USART_ClearFlag
 *
 * @brief		- This function clears status flag
 *
 * @param[in]		- Base address of the U(S)ART peripheral
 * @param[in]		- Name of flag
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************/
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t FlagName)
{
	pUSARTx->SR &= ~(1 << FlagName);
}


/******************************************************************
 * @fn      	      - USART_SendData
 *
 * @brief             - This function sends data over U(S)ART
 *
 * @param[in]         - Pointer to U(S)ART Handle Structure
 * @param[in]         - Pointer to Tx Buffer
 * @param[in]         - Length of Tx Buffer
 *
 * @return            - None
 *
 * @Note	      - None
 *
 *****************************************************************/
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Length)
{
	uint16_t *pdata;

	/* Looping over until "Length" of bytes are transferred */
	for(uint32_t i = 0 ; i < Length; i++)
	{
		/* Waiting till TXE flag is set in SR */
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TXE));

		/* Checking USART_WordLength item for 9BIT or 8BIT in a frame */
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			/* If 9BIT, load DR with 2 bytes masking the bits other than first 9 bits */
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

			/* Checking for USART ParityControl */
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				/* No parity is used in this transfer. 9bits of user data will be sent */
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				/* Parity bit is used in this transfer. 8bits of user data will be sent */
				pTxBuffer++;
			}
		}
		else
		{
			/* 8bit data transfer */
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer  & (uint8_t)0xFF);

			/* Increment buffer address */
			pTxBuffer++;
		}
	}

	/* Waiting till TC flag is set in SR */
	while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TC));
}


/******************************************************************
 * @fn      	      - USART_ReceiveData
 *
 * @brief             - This function receives data over U(S)ART
 *
 * @param[in]         - Pointer to U(S)ART Handle Structure
 * @param[in]         - Pointer to Rx Buffer
 * @param[in]         - Length of Rx Buffer
 *
 * @return            - None
 *
 * @Note 	      - None
 *
 *****************************************************************/
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Length)
{
	/* Looping over until "Length" of bytes are transferred */
	for(uint32_t i = 0 ; i < Length; i++)
	{
		/* Waiting till TXE flag is set in SR */
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RXNE));

		/* Checking USART_WordLength item for 9BIT or 8BIT in a frame */
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			/* Receiving 9bit data in frame */

			/* Checking if USART_ParityControl control is used or not */
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				/* No parity is used. 9bits will be of user data */

				/* Reading only first 9 bits. Masking DR with 0x01FF */
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

				/* Incrementing pRxBuffer address */
				pRxBuffer++;
				pRxBuffer++;
			}
			else
			{
				/* Parity is used. 8bits will be user data and 1 bit is parity */
				/* Reading only 7 bits. Masking DR with 0xFF */
				 *pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);

				 /* Incrementing pRxBuffer address */
				 pRxBuffer++;
			}
		}
		else
		{
			/* Receiving 8bit data in a frame */
			/* Checking if USART_ParityControl is used or not */
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				/* No parity is used. 9bits will be of user data */
				/* Reading 8 bits from DR */
				 *pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
			}

			else
			{
				/* Parity is used. 7bits will be user data and 1 bit is parity */
				/* Reading only 7 bits. Masking DR with 0x7F */
				 *pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0x7F);
			}

			/* Incrementing Rx Buffer */
			pRxBuffer++;
		}
	}
}


/******************************************************************
 * @fn      	      - USART_SendDataInterrupt
 *
 * @brief             - This function sends data over U(S)ART
 * 		        in interrupt mode
 *
 * @param[in]         - Pointer to U(S)ART Handle Structure
 * @param[in]         - Pointer to Tx Buffer
 * @param[in]         - Length of Tx Buffer
 *
 * @return            - Tx state
 *
 * @Note 	      - None
 *
 *****************************************************************/
uint8_t USART_SendDataInterrupt(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Length)
{
	uint8_t txstate = pUSARTHandle->TxBusyState;

	if(txstate != USART_BUSY_IN_TX)
	{
		pUSARTHandle->TxLen = Length;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

		/* Enabling interrupt for TXE */
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE);

		/* Enabling interrupt for TC */
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TCIE);
	}

	return txstate;

}


/******************************************************************
 * @fn      	      - USART_ReceiveDataInterrupt
 *
 * @brief             - This function receives data over U(S)ART
 * 			in interrupt mode
 *
 * @param[in]         - Pointer to U(S)ART Handle Structure
 * @param[in]         - Pointer to Rx Buffer
 * @param[in]         - Length of Rx Buffer
 *
 * @return            - Rx state
 *
 * @Note 	      - None
 *
 *****************************************************************/
uint8_t USART_ReceiveDataInterrupt(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Length)
{
	uint8_t rxstate = pUSARTHandle->RxBusyState;

	if(rxstate != USART_BUSY_IN_RX)
	{
		pUSARTHandle->RxLen = Length;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

		/* Enabling RXNE interrupt */
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);
	}

	return rxstate;
}


/*
 * IRQ Configuration and ISR handling
 */
/*****************************************************************
 * @fn			- USART_IRQInterruptConfig
 *
 * @brief		- This function configures interrupt
 *
 * @param[in]		- IRQ Interrupt number
 * @param[in]		- Macro: Enable/Disable
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************/
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			/* Program ICER1 register (32 to 63) */
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			/* Program ICER2 register (64 to 95) */
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}
}


/*****************************************************************
 * @fn			- USART_IRQPriorityConfig
 *
 * @brief		- This function configures interrupt priority
 *
 * @param[in]		- IRQ Interrupt number
 * @param[in]		- IRQ interrupt priority
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************/
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}


/*****************************************************************
 * @fn			- USART_IRQHandling
 *
 * @brief		- Interrupt handling for diff. U(S)ART events
 *
 * @param[in]		- Pointer to U(S)ART Handle structure
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************/
void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{
	uint32_t temp1 , temp2, temp3;

	uint16_t *pdata;

	/* Checking for TC Flag */
	/* Checking state of TC bit in the SR */
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TC);

	/* Checking state of TCEIE bit */
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TCIE);

	if(temp1 && temp2)
	{
		/* TC caused Interrupt */

		/* Closing transmission and calling application callback if TxLen is zero */
		if (pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			/* Checking TxLen. If it's zero, close data transmission */
			if(!pUSARTHandle->TxLen)
			{
				/* Clearing TC flag */
				pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_TC);

				/* Clearing TCIE bit */
				//pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TCIE);

				/* Resetting application state */
				pUSARTHandle->TxBusyState = USART_READY;

				/* Resetting buffer address to NULL */
				pUSARTHandle->pTxBuffer = NULL;

				/* Resetting length to zero */
				pUSARTHandle->TxLen = 0;

				/* Calling application callback with event USART_EVENT_TX_CMPLT */
				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_TX_CMPLT);
			}
		}
	}

	/* Checking for TXE flag */
	/* Checking state for TXE bit in the SR */
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TXE);

	/* Checking state for TXEIE bit in the CR1 */
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TXEIE);

	if(temp1 && temp2)
	{
		/* TXE caused interrupt */

		if(pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			/* Keep sending data till TxLen reaches zero */
			if(pUSARTHandle->TxLen > 0)
			{
				/* Checking the USART_WordLength item for 9BIT or 8BIT in a frame */
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					/* If 9BIT load the DR with 2 bytes masking the bits other than first 9 bits */
					pdata = (uint16_t*)pUSARTHandle->pTxBuffer;
					pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

					/* Checking for USART_ParityControl */
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						/* No parity used in this transfer. 9 bits of user data will be sent */
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen-=2;
					}
					else
					{
						/* Parity used in this transfer. 8 bits of user data will be sent.
						   9th bit will be replaced by parity bit by the hardware        */
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen-=1;
					}
				}
				else
				{
					/* 8bit data transfer */
					pUSARTHandle->pUSARTx->DR = (*pUSARTHandle->pTxBuffer & (uint8_t)0xFF);

					/* Incrementing buffer address */
					pUSARTHandle->pTxBuffer++;
					pUSARTHandle->TxLen-=1;
				}
			}
			if (pUSARTHandle->TxLen == 0 )
			{
				/* Tx Len is zero. Clearing TXEIE bit (disabling interrupt for TXE flag) */
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_TXEIE);
			}
		}
	}

	/* Checking for RXNE flag */
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_RXNE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_RXNEIE);

	if(temp1 && temp2)
	{
		/* RXNE caused interrupt */
		if(pUSARTHandle->RxBusyState == USART_BUSY_IN_RX)
		{
			if(pUSARTHandle->RxLen > 0)
			{
				/* Checking the USART_WordLength item for 9BIT or 8BIT in a frame */
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					/* Receiving 9bit data in a frame */

					/* Checking if USART_ParityControl control is used or not */
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						/* No parity is used. All 9bits will be of user data */

						/* Reading only first 9 bits. Masking DR with 0x01FF */
						*((uint16_t*) pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->RxLen-=2;
					}
					else
					{
						/* Parity is used. 8bits will be of user data and 1 bit is parity */
						 *pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
						 pUSARTHandle->pRxBuffer++;
						 pUSARTHandle->RxLen-=1;
					}
				}
				else
				{
					/* Receiving 8bit data in a frame */

					/* Checking if USART_ParityControl control is used or not */
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						/* No parity is used. All 9bits will be of user data */

						/* Reading 8 bits from DR */
						 *pUSARTHandle->pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
					}
					else
					{
						/* Parity is used. 7 bits will be of user data and 1 bit is parity */

						/* Reading only 7 bits. Masking DR with 0x7F */
						 *pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);
					}

					/* Incrementing the pRxBuffer */
					pUSARTHandle->pRxBuffer++;
					pUSARTHandle->RxLen-=1;
				}
			}

			if(!pUSARTHandle->RxLen)
			{
				/* Disabling the RXNE */
				pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_RXNEIE );
				pUSARTHandle->RxBusyState = USART_READY;
				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_RX_CMPLT);
			}
		}
	}

	/* Checking for CTS flag. CTS feature is not applicable for UART4 and UART5 */
	/* Checking status of CTS bit in the SR */
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_CTS);

	/* Checking state of CTSE bit in CR3 */
	temp2 = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_CTSE);

	/* Checking state of CTSIE bit in CR3 (This bit is not available for UART4 & UART5.) */
	temp3 = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_CTSIE);

	if(temp1 && temp2 && temp3)
	{
		/* Clearing CTS flag in SR */
		pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_CTS);

		/* CTS caused interrupt */
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_CTS);
	}

	/* Checking for IDLE detection flag */
	/* Checking status of IDLE flag bit in SR */
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_IDLE);

	/* Checking state of IDLEIE bit in CR1 */
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_IDLEIE);

	if(temp1 && temp2)
	{
		/* Clearing IDLE flag. Referring to the RM to understand the clear sequence */
		temp1 = pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_IDLE);

		/* IDLE caused interrupt */
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_IDLE);
	}

	/* Checking for OverRun detection flag */

	/* Checking status of ORE flag in the SR */
	temp1 = pUSARTHandle->pUSARTx->SR & USART_SR_ORE;

	/* Checking status of RXNEIE  bit in the CR1 */
	temp2 = pUSARTHandle->pUSARTx->CR1 & USART_CR1_RXNEIE;
	if(temp1  && temp2)
	{
		/* OverRun error caused interrupt */
		USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_ORE);
	}

	/* Checking for Error Flag */
	/* Noise Flag, OverRun Error and Framing Error  in MultiBuffer Communication */
	/* This part of code will be executed only if MultiBuffer mode is used 		 */
	temp2 =  pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_EIE) ;

	if(temp2 )
	{
		temp1 = pUSARTHandle->pUSARTx->SR;
		if(temp1 & (1 << USART_SR_FE))
		{
			/*
				This bit is set by hardware when a de-synchronization, excessive noise or a break character
				is detected. It is cleared by a software sequence (an read to the USART_SR register
				followed by a read to the USART_DR register).
			*/
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_FE);
		}

		if(temp1 & (1 << USART_SR_NF))
		{
			/*
				This bit is set by hardware when noise is detected on a received frame. It is cleared by a
				software sequence (an read to the USART_SR register followed by a read to the
				USART_DR register).
			*/
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_NE);
		}

		if(temp1 & ( 1 << USART_SR_ORE) )
		{
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_ORE);
		}
	}
}


/*******************************************************************
 * @fn      	      - USART_SetBaudRate
 *
 * @brief             - This function sets U(S)ART Baudrate
 *
 * @param[in]         - Base address of the U(S)ART peripheral
 * @param[in]         - Baud rate value
 *
 * @return            - None
 *
 * @Note              - None
 *
 ********************************************************************/
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{
	/* Holds APB clock */
	uint32_t PCLKx;

	uint32_t usartdiv;

	/* Hold Mantissa and Fraction values */
	uint32_t M_part,F_part;

	uint32_t tempreg = 0;

	/* Getting value of APB bus clock into the variable PCLKxn */
	if(pUSARTx == USART1 || pUSARTx == USART6)
	{
		/* USART1 and USART6 are hanging on APB2 bus */
		PCLKx = RCC_GetPCLK2Value();
	}
	else
	{
		PCLKx = RCC_GetPCLK1Value();
	}

	/* Checking for OVER8 configuration bit */
	if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
	{
		/* OVER8 = 1. Over sampling by 8 */
		usartdiv = ((25 * PCLKx) / (2 * BaudRate));
	}
	else
	{
		/* OVER8 = 0. Over sampling by 16 */
		usartdiv = ((25 * PCLKx) / (4 * BaudRate));
	}

	/* Calculating Mantissa part */
	M_part = usartdiv/100;

	/* Placing Mantissa part in appropriate bit position. USART_BRR */
	tempreg |= M_part << 4;

	/* Extracting fractional part */
	F_part = (usartdiv - (M_part * 100));

	/* Calculating final fractional part */
	if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
	{
		/* OVER8 = 1. Over sampling by 8 */
		F_part = (((F_part * 8) + 50) / 100) & ((uint8_t)0x07);
	}
	else
	{
		/* OVER8 = 0. Over sampling by 16 */
		F_part = ((( F_part * 16) + 50) / 100) & ((uint8_t)0x0F);
	}

	/* Placing fractional part in appropriate bit position. USART_BRR */
	tempreg |= F_part;

	/* Copying value of tempreg in to BRR register */
	pUSARTx->BRR = tempreg;
}


/*****************************************************************
 * @fn			- USART_ApplicationEventCallback
 *
 * @brief		- Application event callback function
 *
 * @param[in]		- Handle structure
 * @param[in]		- Application event
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************/
__weak void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t AppEvent)
{
	/* This is a week implementation. The application may override this function. */
}
