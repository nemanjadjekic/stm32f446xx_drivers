/*
 * 006_SPI_SendData.c
 *
 *  Created on: Jan 29, 2020
 *      Author: nemanja
 */

#include <string.h>
#include "stm32f446xx.h"

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
    SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;
    SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

    /* SCLK Init */
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GPIO_Init(&SPIPins);

    /* MOSI Init */
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
    GPIO_Init(&SPIPins);

    /* MISO Init */
    //SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
    //GPIO_Init(&SPIPins);

    /* NSS Init */
    //SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    //GPIO_Init(&SPIPins);

}


void SPI2_Inits()
{
    SPI_Handle_t SPIHandle;

    SPIHandle.pSPIx = SPI2;
    SPIHandle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
    SPIHandle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
    SPIHandle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2; //8MHz
    SPIHandle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
    SPIHandle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
    SPIHandle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
    SPIHandle.SPIConfig.SPI_SSM = SPI_SSM_EN; //SSM Enabled for NSS pin

    SPI_Init(&SPIHandle);
}


int main(void)
{
    char user_data[] = "Hello World!";

    /* Initialize GPIO pins to behave as SPI2 pins */
    SPI2_GPIOInits();

    /* Initialize SPI2 peripheral parameters */
    SPI2_Inits();

    /* Makes NSS sigmal internaly high and avoids MODE error */
    SPI_SSIConfig(SPI2, ENABLE);

    /* Enable SPI2 peripheral */
    SPI_PeripheralControl(SPI2, ENABLE);

    /* Send SPI data */
    SPI_SendData(SPI2, (uint8_t *)user_data, strlen(user_data));

    /* Confirm SPI2 not busy */
    SPI_GetFlagStatus(SPI2, SPI_SR_BSY);

    /* Disable SPI2 peripheral */
    SPI_PeripheralControl(SPI2, DISABLE);

    while(1);

    return 0;
}
