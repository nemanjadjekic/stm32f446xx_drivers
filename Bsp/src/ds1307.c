/*
 * ds1307.c
 *
 *  Created on: Feb 1, 2023
 *      Author: nemanja
 */

#include "ds1307.h"

static void DS1307_I2C_Config(void);
static void DS1307_I2C_PinConfig(void);
static void DS1307_Write(uint8_t Value, uint8_t RegAddr);
static uint8_t DS1307_Read(uint8_t RegAddr);

I2C_Handle_t g_DS1307_I2C_Handle;


uint8_t DS1307_Init(void)
{
	// Initialize I2C Pins
	DS1307_I2C_PinConfig();

	// Initialize I2C Peripherals
	DS1307_I2C_Config();

	// Enable I2C Peripherals
	I2C_PeripheralControl(DS1307_I2C, ENABLE);

	// Make clock halt = 0
	DS1307_Write(0x00, DS1307_ADDR_SEC);

	// Read back clock halt bit
	uint8_t clock_state = DS1307_Read(DS1307_ADDR_SEC);

	return ((clock_state >> 7 ) & 0x1);
}

void DS1307_SetCurrentTime(RTC_Time_t *RTC_Time)
{

}

 void DS1307_GetCurrentTime(RTC_Time_t *RTC_Time)
{

}

void DS1307_SetCurrentDate(RTC_Date_t *RTC_Date)
{

}

void DS1307_GetCurrentDate(RTC_Date_t *RTC_Date)
{

}

static void DS1307_I2C_PinConfig(void)
{
	GPIO_Handle_t I2CPins;

	memset(&I2CPins, 0, sizeof(I2CPins));
	/*
	 * I2C1_SCL ==> PB6
	 * IC21_SDA ==> PB7
	 */
	I2CPins.pGPIOx = DS1307_I2C_GPIO_PORT;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = DS1307_I2C_PUPD;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	/* SCL */
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = DS1307_I2C_SCL_PIN;
	GPIO_Init(&I2CPins);

	/* SDA */
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = DS1307_I2C_SDA_PIN;
	GPIO_Init(&I2CPins);
}

static void DS1307_I2C_Config(void)
{
	g_DS1307_I2C_Handle.pI2Cx = DS1307_I2C;
	g_DS1307_I2C_Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	g_DS1307_I2C_Handle.I2C_Config.I2C_SCLSpeed = DS1307_I2C_SPEED;
	I2C_Init(&g_DS1307_I2C_Handle);
}

static void DS1307_Write(uint8_t Value, uint8_t RegAddr)
{
	uint8_t tx[2];
	tx[0] = RegAddr;
	tx[1] = Value;
	I2C_MasterSendData(&g_DS1307_I2C_Handle, tx, 2, DS1307_I2C_ADDRESS, 0);
}

static uint8_t DS1307_Read(uint8_t RegAddr)
{
	uint8_t data;
	I2C_MasterSendData(&g_DS1307_I2C_Handle, &RegAddr, 1, DS1307_I2C_ADDRESS, 0);
	I2C_MasterReceiveData(&g_DS1307_I2C_Handle, &data, 1, DS1307_I2C_ADDRESS, 0);

	return data;
}
