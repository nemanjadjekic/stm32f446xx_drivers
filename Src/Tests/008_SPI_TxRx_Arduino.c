/*
 * 008_SPI_TxRx_Arduino.c
 *
 *  Created on: Feb 5, 2020
 *      Author: nemanja
 */

#include <stm32f446xx.h>
#include <string.h>

/*
 * Alternate functionality
 *
 * PB15 --> SPI2_MOSI
 * PB14 --> SPI2_MISO
 * PB13 --> SPI2_SLCK
 * PB12 --> SPI2_NSS
 * ALT function mode: AF5
 */

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

void delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}


uint8_t SPI_VerifyResponse(uint8_t ackByte)
{
	if(ackByte == 0xF5)
	{
		return 1;
	}

	return 0;
}


int main(void)
{
	uint8_t dummyWrite = 0xFF;
	uint8_t dummyRead;

	/* Initialize button */
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

	while(1)
	{
		/* Wait till button is pressed */
		while( GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) );

		printf("\nSPI communication started!");

		/* 200ms delay */
		delay();

		/* Enable SPI2 peripheral */
		SPI_PeripheralControl(SPI2, ENABLE);

		/* Send SPI data: CMD LED Control */
		uint8_t commandCode = COMMAND_LED_CTRL;

		/* Send command */
		SPI_SendData(SPI2, &commandCode, 1);

		/* Dummy read to clear off the RXNE */
		SPI_ReceiveData(SPI2, &dummyRead, 1);

		/* Send dummy bits (byte) to fetch the response from slave */
		SPI_SendData(SPI2, &dummyWrite, 1);

		/* Receive Acknowledgment byte */
		uint8_t ackByte;
		SPI_ReceiveData(SPI2, &ackByte, 1);

		/* Verify response from SPI slave */
		uint8_t args[2];
		if( SPI_VerifyResponse(ackByte) )
		{
			/* Send arguments */
			args[0] = LED_PIN;
			args[1] = LED_ON;
			SPI_SendData(SPI2, args, 2);
			printf("Command LED executed!\n");
		}
		/* End of CMD LED Control */

		/* Send SPI data: CMD Sensor Read */
		commandCode = COMMAND_SENSOR_READ;

		/* Send command */
		SPI_SendData(SPI2, &commandCode, 1);

		/* Dummy read to clear off the RXNE */
		SPI_ReceiveData(SPI2, &dummyRead, 1);

		/* Send dummy bits (byte) to fetch the response from slave */
		SPI_SendData(SPI2, &dummyWrite, 1);

		/* Receive Acknowledgment byte */
		SPI_ReceiveData(SPI2, &ackByte, 1);

		/* Verify response from SPI slave */
		if( SPI_VerifyResponse(ackByte) )
		{
			/* Send arguments */
			args[0] = ANALOG_PIN0;
			SPI_SendData(SPI2, args, 1);
			printf("Command Sensor Read executed!\n");

			/* Dummy read to clear off the RXNE */
			SPI_ReceiveData(SPI2, &dummyRead, 1);

			/* Send dummy bits (byte) to fetch the response from slave */
			SPI_SendData(SPI2, &dummyWrite, 1);

			/* Wait for Slave to be ready with data */
			delay();

			/* Receive Sensor Data from Slave */
			uint8_t analogRead;
			SPI_ReceiveData(SPI2, &analogRead, 1);
			printf("Analog read value: %d\n",analogRead);
		}
		/* End of CMD Sensor Read */

		/* Confirm SPI2 not busy */
		SPI_GetFlagStatus(SPI2, SPI_SR_BSY);

		/* Disable SPI2 peripheral */
		SPI_PeripheralControl(SPI2, DISABLE);

		//TODO: Implement rest of commands to test if needed
		printf("\nSPI communication closed!");
	}

	return 0;
}
