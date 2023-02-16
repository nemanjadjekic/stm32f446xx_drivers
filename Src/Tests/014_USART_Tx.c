/*
 * 014_USART_Tx.c
 *
 *  Created on: Mar 1, 2020
 *      Author: nemanja
 */

#include <stdio.h>
#include <string.h>
#include <stm32f446xx.h>

USART_Handle_t USART3Handle;

/* Transmitter buffer */
char msg[32] = "STM32 U(S)ART testing..\n";


void USART3_Init(void)
{
    USART3Handle.pUSARTx = USART3;
    USART3Handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
    USART3Handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
    USART3Handle.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
    USART3Handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
    USART3Handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
    USART3Handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;

    USART_Init(&USART3Handle);
}


/*
 * PC10-> TX
 * PC11-> RX
 */
void USART3_GPIOInit(void)
{
    GPIO_Handle_t USART3Pins;

    USART3Pins.pGPIOx = GPIOC;
    USART3Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    USART3Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
    USART3Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
    USART3Pins.GPIO_PinConfig.GPIO_PinAltFunMode = 7;
    USART3Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

    /* TX */
    USART3Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
    GPIO_Init(&USART3Pins);

    /* RX */
    USART3Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_11;
    GPIO_Init(&USART3Pins);
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


void delay(void)
{
    for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}


int main(void)
{
    /* Button GPIO Initialization */
    GPIO_ButtonInit();

    /* USART3 Pins Initialization */
    USART3_GPIOInit();

    /* USART3 Peripherals Initialization */
    USART3_Init();

    /* Enabling USART3 Peripherals */
    USART_PeripheralControl(USART3,ENABLE);

    while(1)
    {
        /* Wait till button is pressed */
        while( GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) );

        /* De-bouncing protection */
        delay();

        /* Sending data over USART */
        USART_SendData(&USART3Handle, (uint8_t*)msg, strlen(msg));
    }

    return 0;
}
