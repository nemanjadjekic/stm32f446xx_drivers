/*
 * stm32f466xx_rcc_driver.c
 *
 *  Created on: Feb 29, 2020
 *      Author: nemanja
 */

#include <stm32f466xx_rcc_driver.h>

uint16_t AHB_Prescaler[8] = {2,4,8,16,64,128,256,512};
uint8_t  APBx_Prescaler[4] = {2,4,8,16};

/*****************************************************************
 * @fn			- RCC_GetPCLK1Value
 *
 * @brief		- This function returns PClock 1 value
 *
 *
 * @return		- PClock 1 value
 *
 * @Note		- None
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

	if(temp < 4)
	{
		apb1p = 1;
	}
	else
	{
		apb1p = APBx_Prescaler[temp-4];
	}

	pclk1 = (SystemClk / ahbp) / apb1p;

	return pclk1;
}


/*****************************************************************
 * @fn			- RCC_GetPCLK2Value
 *
 * @brief		- This function returns PClock 2 value
 *
 *
 * @return		- PClock 2 value
 *
 * @Note		- None
 *
 *****************************************************************/
uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t SystemClk = 0, temp, pclk2;
	uint8_t clksrc = ((RCC->CFGR >> 2) & 0x3);
	uint8_t ahbp,apb2p;

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

	/* APB2 */
	temp = ((RCC->CFGR >> 13) & 0x7);

	if(temp < 4)
	{
		apb2p = 1;
	}
	else
	{
		apb2p = APBx_Prescaler[temp-4];
	}

	pclk2 = (SystemClk / ahbp) / apb2p;

	return pclk2;
}


/*****************************************************************
 * @fn			- RCC_GetPLLOutputClock
 *
 * @brief		- This function returns PLL output value
 *
 *
 * @return		- PLL output value
 *
 * @Note		- None
 *
 *****************************************************************/
uint32_t RCC_GetPLLOutputClock(void)
{
	/* Not used for now */
	return 0;
}
