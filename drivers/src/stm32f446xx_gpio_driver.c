/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: Aug 28, 2019
 *      Author: nemanja
 */

#include "stm32f446xx_gpio_driver.h"

/*
 * Peripheral Clock setup
 */

/*****************************************************************
 * @fn				- GPIO_PeriClockControl
 *
 * @brief			- This function enables or disables peripheral
 * 					  clock for the given GPIO port
 *
 * @param[in]		- Base address of the GPIO peripheral
 * @param[in]		- Macros:	Enable or Disable
 *
 * @return			- None
 *
 * @Note			- None
 *
 *****************************************************************/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
	}
}


/*
 * Init and De-Init
 */
/*****************************************************************
 * @fn				- GPIO_Init
 *
 * @brief			- This function initialize GPIO peripherals
 *
 * @param[in]		- Pointer to GPIO Handle structure
 *
 * @return			- None
 *
 * @Note			- None
 *
 *****************************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;

	/* Enable peripheral clock */
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	/* GPIO pin mode configuration */
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		/* Non interrupt mode */
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER  &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //Clearing pin
		pGPIOHandle->pGPIOx->MODER |= temp;//Setting pin
		temp = 0;
	}
	else
	{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			/* Configure the FTSR */
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			/* Clear corresponding RTSR bit */
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			/* Configure the RTSR */
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			/* Clear corresponding FTSR bit */
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FRT)
		{
			/* Configure the FTSR */
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			/* Configure the RTSR */
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		/* Configure GPIO port selection in SYSCFG_EXTICR */
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);

		/* Enable EXTI interrupt delivery using IMR */
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	/* GPIO pin speed configuration */
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR  &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	/* GPIO pull up/down settings configuration */
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR  &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	/* GPIO output type configuration */
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER  &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	/* GPIO alt functionality configuration */
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		/* Alt functionality registers configuration */
		uint8_t afrBitNum, afrLowHighReg;

		afrLowHighReg = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		afrBitNum = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[afrLowHighReg] &= ~(0xF << (4 * afrBitNum));
		pGPIOHandle->pGPIOx->AFR[afrLowHighReg] |= pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * afrBitNum);
	}
}


/*****************************************************************
 * @fn				- GPIO_DeInit
 *
 * @brief			- This function de-initialize GPIO peripherals
 *
 * @param[in]		- Base address of the GPIO peripheral
 *
 * @return			- None
 *
 * @Note			- None
 *
 *****************************************************************/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
}


/*
 * Data read and write
 */
/*****************************************************************
 * @fn				- GPIO_ReadFromInputPin
 *
 * @brief			- This function reads value of input pin, on
 * 					  a specific port
 *
 * @param[in]		- Base address of the GPIO peripheral
 * @param[in]		- Pin number
 *
 * @return			- Content of the input data
 *
 * @Note			- 0 or 1
 *
 *****************************************************************/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;

	value = (uint8_t )((pGPIOx->IDR >> PinNumber) & 0x00000001 );

	return value;
}


/*****************************************************************
 * @fn				- GPIO_ReadFromInputPort
 *
 * @brief			- This function reads value of input port
 *
 * @param[in]		- Base address of the GPIO peripheral
 *
 * @return			- Content of the input data
 *
 * @Note			- None
 *
 *****************************************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;

	value = (uint16_t )pGPIOx->IDR;

	return value;
}


/*****************************************************************
 * @fn				- GPIO_WriteToOutputPin
 *
 * @brief			- This function writes value on a specific
 * 					  output pin
 *
 * @param[in]		- Base address of the GPIO peripheral
 * @param[in]		- Pin number
 * @param[in]		- Value (Set/Reset Macro)
 *
 * @return			- None
 *
 * @Note			- None
 *
 *****************************************************************/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		/* Write 1 to the output data register at the bit field corresponding to the pin number */
		pGPIOx->ODR |= ( 1 << PinNumber );
	}
	else
	{
		/* Write 0 to the output data register at the bit field corresponding to the pin number */
		pGPIOx->ODR &= ~( 1 << PinNumber );	//Clear pin
	}
}


/*****************************************************************
 * @fn				- GPIO_WriteToOutputPort
 *
 * @brief			- This function writes value on a specific
 * 					  output port
 *
 * @param[in]		- Base address of the GPIO peripheral
 * @param[in]		- Value (Set/Reset Macro)
 *
 * @return			- None
 *
 * @Note			- None
 *
 *****************************************************************/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}


/*****************************************************************
 * @fn				- GPIO_ToggleOutputPin
 *
 * @brief			- This function toggles specific output pin
 *
 * @param[in]		- Base address of the GPIO peripheral
 * @param[in]		- Pin number
 *
 * @return			- None
 *
 * @Note			- None
 *
 *****************************************************************/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}


/*
 * IRQ Configuration and ISR handling
 */
/*****************************************************************
 * @fn				- GPIO_IRQConfig
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
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			/* Program ICER1 register (32 to 63) */
		}
		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			/* Program ICER2 register (64 to 95) */
		}
	}
}


/*****************************************************************
 * @fn				- GPIO_IRQPriorityConfig
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
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}


/*****************************************************************
 * @fn				- GPIO_IRQHandling
 *
 * @brief			- This function handle interrupts
 *
 * @param[in]		- Pin number
 *
 * @return			- None
 *
 * @Note			- None
 *
 *****************************************************************/
void GPIO_IRQHandling(uint8_t PinNumber)
{
	/* Clear the PR register corresponding to pin number */
	if(EXTI->PR & (1 << PinNumber))
	{
		/* Clear pin */
		EXTI->PR |= (1 << PinNumber);
	}
}


