/*
 * 013_I2C_SlaveTxRx.c
 *
 *  Created on: Feb 29, 2020
 *      Author: nemanja
 */

#include <stdio.h>
#include <string.h>
#include <stm32f446xx.h>


#define SLAVE_ADDR  0x68
#define MY_ADDR SLAVE_ADDR

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}


I2C_Handle_t I2C1Handle;

/* Transmitter buffer */
uint8_t tx_buff[32] = "STM32 Slave Mode testing..";


/*
 * PB6-> SCL
 * PB7-> SDA
 */
void I2C1_GPIOInits(void)
{
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	/* SCL */
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);


	/* SDA */
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&I2CPins);
}


void I2C1_Inits(void)
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);
}


void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GpioBtn,GpioLed;

	/* BTN GPIO Config */
	GpioBtn.pGPIOx = GPIOC;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;

	GPIO_Init(&GpioBtn);

	/* LED GPIO Config */
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&GpioLed);
}


int main(void)
{
	printf("Application is running\n");

	GPIO_ButtonInit();

	/* I2C Pins init */
	I2C1_GPIOInits();

	/* I2C Peripheral init */
	I2C1_Inits();

	/* I2C IRQ configurations */
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV,ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER,ENABLE);

	/* I2C Enable/Disable Slave Callback events */
	I2C_SlaveEnableDisableCallbackEvents(I2C1,ENABLE);

	/* Enable I2C peripheral */
	I2C_PeripheralControl(I2C1,ENABLE);

	/* ACK bit is made 1 after PE=1 */
	I2C_ManageAcking(I2C1,I2C_ACK_ENABLE);

	while(1);
}


void I2C1_EV_IRQHandler (void)
{
	I2C_EV_IRQHandling(&I2C1Handle);
}


void I2C1_ER_IRQHandler (void)
{
	I2C_ER_IRQHandling(&I2C1Handle);
}


void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEvent)
{
	static uint8_t commandCode = 0;
	static uint8_t count = 0;

	if(AppEvent == I2C_EV_DATA_REQ)
	{
		/* Master sends request for data. Slave sends data to master. */
		if(commandCode == 0x51)
		{
			/* Sends data length information to the master */
			I2C_SlaveSendData(pI2CHandle->pI2Cx, strlen((char*)tx_buff));
		}
		else if(commandCode == 0x52)
		{
			/* Sends contents of tx_buff to the master */
			I2C_SlaveSendData(pI2CHandle->pI2Cx, tx_buff[count++]);
		}
	}
	else if(AppEvent == I2C_EV_DATA_RCV)
	{
		/* Slave is reading data */
		commandCode = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);

	}
	else if(AppEvent == I2C_ERROR_AF)
	{
		/* Master sends NACK. It doesn't need more data. */
		commandCode = 0xFF;
		count = 0;
	}
	else if(AppEvent == I2C_EV_STOP)
	{
		/* Master ended I2C communication with slave */
	}
}


