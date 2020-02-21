/*
 * stm32f446xx_i2c_driver.c
 *
 *  Created on: Feb 13, 2020
 *      Author: nemanja
 */

#include <stm32f446xx_gpio_driver.h>

uint16_t AHB_Prescaler[8] = {2,4,8,16,32,64,128,256,512};
uint8_t  APB1_Prescaler[4] = {2,4,8,16};

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressesPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearAddrFlag(I2C_RegDef_t *pI2Cx);

/*
 * Peripheral Clock setup
 */
/*****************************************************************
 * @fn				- I2C_PeriClockControl
 *
 * @brief			- This function enables or disables peripheral
 * 					  clock for the given I2C port
 *
 * @param[in]		- Base address of the I2C peripheral
 * @param[in]		- Macros:	Enable or Disable
 *
 * @return			- None
 *
 * @Note			- None
 *
 *****************************************************************/
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
		else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
		else if(pI2Cx == SPI3)
		{
			I2C3_PCLK_EN();
		}
	}
	else
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}
		else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}
		else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}
	}
}


/*****************************************************************
 * @fn				- RCC_GetPLLOutputClock
 *
 * @brief			- This function returns PLL output value
 *
 *
 * @return			- PLL output value
 *
 * @Note			- None
 *
 *****************************************************************/
uint32_t RCC_GetPLLOutputClock(void)
{
	/* Not used for now */
	return 0;
}

/*****************************************************************
 * @fn				- RCC_GetPCLK1Value
 *
 * @brief			- This function returns PClock 1 value
 *
 *
 * @return			- PClock 1 value
 *
 * @Note			- None
 *
 *****************************************************************/
uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1, SystemClk;
	uint8_t clksrc, temp, ahbp, apb1p;

	clksrc = ((RCC->CFGR >> 2) & 0x3);

	if(clksrc == 0)
	{
		SystemClk = 16000000;
	}
	else if(clksrc == 1)
	{
		SystemClk = 8000000;
	}
	else if(clksrc == 2)
	{
		SystemClk = RCC_GetPLLOutputClock();
	}

	/* AHBP */
	temp = ((RCC->CFGR >> 4) & 0xF);

	if(temp < 8)
	{
		ahbp = 1;
	}
	else
	{
		ahbp = AHB_Prescaler[temp-8];
	}

	/* APB1 */
	temp = ((RCC->CFGR >> 10) & 0x7);

	if(temp < 8)
	{
		apb1p = 1;
	}
	else
	{
		apb1p = APB1_Prescaler[temp-4];
	}

	pclk1 = (SystemClk / ahbp) / apb1p;

	return pclk1;
}


/*
 * Init and De-Init
 */
/*****************************************************************
 * @fn				- I2C_Init
 *
 * @brief			- This function initialize I2C peripherals
 *
 * @param[in]		- Pointer to I2C Handle structure
 *
 * @return			- None
 *
 * @Note			- None
 *
 *****************************************************************/
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0;

	/* ACK control bit */
	tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl << 10;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	/* CR2 FREQ filed configuration */
	tempreg|= RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

	/* Programming device own address */
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= (1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	/* CCR calculations */
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		/* Standard mode */
		ccr_value = (RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		tempreg |= (ccr_value & 0xFFF);
	}
	else
	{
		/* Fast mode */
		tempreg |= (1 << 15);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = (RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		else
		{
			ccr_value = (RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	/* TRISE Configuration */
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		/* Standard mode */
		tempreg = (RCC_GetPCLK1Value() / 1000000U) + 1;
	}
	else
	{
		/* Fast mode */
		tempreg = ( (RCC_GetPCLK1Value() * 300) / 1000000U ) + 1;
	}

	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);
}


/*****************************************************************
 * @fn				- I2C_DeInit
 *
 * @brief			- This function de-initialize I2C peripherals
 *
 * @param[in]		- Base address of the I2C peripheral
 *
 * @return			- None
 *
 * @Note			- None
 *
 *****************************************************************/
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if(pI2Cx == I2C1)
	{
		I2C1_REG_RESET();
	}
	else if(pI2Cx == I2C2)
	{
		I2C2_REG_RESET();
	}
	else if(pI2Cx == I2C3)
	{
		I2C3_REG_RESET();
	}
}


/*****************************************************************
 * @fn				- I2C_GetFlagStatus
 *
 * @brief			- This function returns if bit in register is
 * 					  set or not
 *
 * @param[in]		- Base address of the I2C peripheral
 * @param[in]		- Name of flag
 *
 * @return			- Flag status (True/False)
 *
 * @Note			- None
 *
 *****************************************************************/
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if(pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}


/*****************************************************************
 * @fn				- I2C_PeripheralControl
 *
 * @brief			- This function sets I2C peripheral control
 *
 * @param[in]		- Base address of the I2C peripheral
 * @param[in]		- Enable or Disable command
 *
 * @return			- None
 *
 * @Note			- None
 *
 *****************************************************************/
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}
	else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}


/*****************************************************************
 * @fn				- I2C_MasterSendData
 *
 * @brief			- I2C Master sends data to slaves
 *
 * @param[in]		- Handle structure
 * @param[in]		- Transmit buffer
 * @param[in]		- Length of transmit buffer
 * @param[in]		- Slave address
 *
 * @return			- None
 *
 * @Note			- None
 *
 *****************************************************************/
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Length, uint8_t SlaveAddr)
{
	/* Generating start condition */
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	/* Confirming start generation is completed by checking SB flag in SR1 */
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB) );

	/* Send the address of the slave with r/nm bit set to w(0) (total 8 bits) */
	I2C_ExecuteAddressesPhase(pI2CHandle->pI2Cx, SlaveAddr);

	/* Confirming address phase is completed by checking ADDR flag in SR1 */
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR) );

	/* Clearing the address flag according to its software sequence */
	I2C_ClearAddrFlag(pI2CHandle->pI2Cx);

	/* Send the data until length reaches 0 value */
	while(Length > 0)
	{
		/* Waiting untill TX is set */
		while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE) );
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Length--;
	}

	/* Waiting for TXE=1 and BTF=1 before generating STOP condition */
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE) );
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF) );

	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}


/*****************************************************************
 *               Helper functions implementation                 *
 *****************************************************************/
/*****************************************************************
 * @fn				- I2C_GenerateStartCondition
 *
 * @brief			- Generate start condition for I2C
 *
 * @param[in]		- Base address of the I2C peripheral
 *
 * @return			- None
 *
 * @Note			- None
 *
 *****************************************************************/
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}


/*****************************************************************
 * @fn				- I2C_GenerateStopCondition
 *
 * @brief			- Generate stop condition for I2C
 *
 * @param[in]		- Base address of the I2C peripheral
 *
 * @return			- None
 *
 * @Note			- None
 *
 *****************************************************************/
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}


/*****************************************************************
 * @fn				- I2C_ExecuteAddressesPhase
 *
 * @brief			- Send the address of the slave with r/nm
 * 					  bit set to w(0) (total 8 bits)
 *
 * @param[in]		- Base address of the I2C peripheral
 * @param[in]		- Slave address
 *
 * @return			- None
 *
 * @Note			- None
 *
 *****************************************************************/
static void I2C_ExecuteAddressesPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	/* SlaveAddr is Slave address + r/nw bit=1 */
	SlaveAddr |= 1;
	pI2Cx->DR = SlaveAddr;
}


/*****************************************************************
 * @fn				- I2C_ClearAddrFlag
 *
 * @brief			- Clear address flag
 *
 * @param[in]		- Base address of the I2C peripheral
 *
 * @return			- None
 *
 * @Note			- None
 *
 *****************************************************************/
static void I2C_ClearAddrFlag(I2C_RegDef_t *pI2Cx)
{
	uint32_t dummyRead = pI2Cx->SR1;
	dummyRead = pI2Cx->SR2;
	(void)dummyRead;
}


/*****************************************************************
 * @fn				- I2C_ApplicationEventCallback
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
__weak void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEvent)
{
	/* This is a week implementation. The application may override this function. */
}
