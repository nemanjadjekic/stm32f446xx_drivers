/*
 * 009_SPI_TxRx_Arduino_IRQ.c
 *
 *  Created on: Feb 21, 2020
 *      Author: nemanja
 */


#include <stm32f446xx.h>
#include <string.h>


/* Command codes */
#define COMMAND_LED_CTRL		0x50
#define COMMAND_SENSOR_READ		0x51
#define COMMAND_LED_READ		0x52
#define COMMAND_PRINT			0x53
#define COMMAND_ID_READ			0x54

#define LED_ON					1
#define LED_OFF					0

/* Analog pins */
#define ANALOG_PIN0				0
#define ANALOG_PIN1				1
#define ANALOG_PIN2				2
#define ANALOG_PIN3				3
#define ANALOG_PIN4				4

/* Arduino LED */
#define LED_PIN					13


SPI_Handle_t SPI2handle;

uint8_t RcvBuff[100];
uint8_t ReadByte;
uint8_t RxContFlag = RESET;


void delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}


/*
 * Alternate functionality
 *
 * PB15 --> SPI2_MOSI
 * PB14 --> SPI2_MISO
 * PB13 --> SPI2_SLCK
 * PB12 --> SPI2_NSS
 * ALT function mode: AF5
 */
void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	/* SCLK Init */
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	/* MOSI Init */
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	/* MISO Init */
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

	/* NSS Init */
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);

}


void SPI2_Inits(void)
{
	SPI_Handle_t SPIHandle;

	SPIHandle.pSPIx = SPI2;
	SPIHandle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPIHandle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPIHandle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8; //2MHz
	SPIHandle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPIHandle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPIHandle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPIHandle.SPIConfig.SPI_SSM = SPI_SSM_DI; //HW Slave management enabled for NSS pin

	SPI_Init(&SPIHandle);
}


void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GpioBtn;

	GpioBtn.pGPIOx = GPIOC;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;

	//GPIO_PeriClockControl(GPIOC, ENABLE);

	GPIO_Init(&GpioBtn);
}


uint8_t SPI_VerifyResponse(uint8_t ackByte)
{
	if(ackByte == (uint8_t)0xF5)
	{
		return 1;
	}

	return 0;
}


int main(void)
{
	GPIO_ButtonInit();

	/* Initialize GPIO pins to behave as SPI2 pins */
	SPI2_GPIOInits();

	/* Initialize SPI2 peripheral parameters */
	SPI2_Inits();

	/*
	 * Making SSOE 1 does NSS output enable.
	 * The NSS pin is automatically managed by hardware.
	 * i.e. when SPE=1, NSS will be pulled to LOW and
	 * NSS pin will be HIGH when SPE=0
	 */
	SPI_SSOEConfig(SPI2, ENABLE);

	SPI_IRQInterruptConfig(IRQ_NO_SPI2, ENABLE);

	/* Wait till button is pressed */
	while( GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) );

	/* 200ms delay */
	delay();

	/* Enable SPI2 peripheral */
	SPI_PeripheralControl(SPI2, ENABLE);

	/* Set RX flag */
	RxContFlag = SET;

	while(RxContFlag == SET)
	{
		while( ! (SPI_ReceiveDataInterruptMode(&SPI2handle, &ReadByte, 1) == SPI_READY) );
	}

	/* Confirming SPI is not busy */
	while( SPI_GetFlagStatus(SPI2,SPI_FLAG_BUSY) );

	/* Disable the SPI2 peripheral */
	SPI_PeripheralControl(SPI2,DISABLE);

	return 0;
}

void SPI2_IRQHandler(void)
{

	SPI_IRQHandling(&SPI2handle);
}


void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{
	static uint32_t i =0;
	static uint8_t  rcv_start = 0;
	if(AppEv == SPI_EVENT_RX_CMPLT)
	{
		if(ReadByte == 0XF1)
		{
			rcv_start = 1;
		}else
		{
			if(rcv_start)
			{
				if(ReadByte == '\r')
				{
					RxContFlag = RESET;
					rcv_start =0;
					RcvBuff[i++] = ReadByte; //place the \r
					i=0;
				}else
				{
					RcvBuff[i++] = ReadByte;

				}
			}
		}

	}

}
