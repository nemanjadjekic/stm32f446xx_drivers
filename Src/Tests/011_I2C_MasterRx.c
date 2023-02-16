/*
 * 011_I2C_MasterRx.c
 *
 *  Created on: Feb 26, 2020
 *      Author: nemanja
 */

#include<stdio.h>
#include<string.h>
#include <stm32f446xx.h>

extern void initialise_monitor_handles();

#define MY_ADDR     0x61

#define SLAVE_ADDR  0x68

void delay(void)
{
    for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

I2C_Handle_t I2C1Handle;

/* Receive buffer */
uint8_t rcv_buf[32];

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
    I2CPins. GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

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
    uint8_t commandcode;

    uint8_t len;

    initialise_monitor_handles();

    printf("Application is running\n");

    GPIO_ButtonInit();

    /* I2C Pins init */
    I2C1_GPIOInits();

    /* I2C Peripheral init */
    I2C1_Inits();

    /* Enable I2C peripheral */
    I2C_PeripheralControl(I2C1,ENABLE);

    /* ACK bit is made 1 after PE=1 */
    I2C_ManageAcking(I2C1,I2C_ACK_ENABLE);

    while(1)
    {
        printf("Application is running\n");
        /* Wait till button is pressed */
        while( GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) );

        /* De-bouncing protection */
        delay();

        commandcode = 0x51;

        I2C_MasterSendData(&I2C1Handle,&commandcode,1,SLAVE_ADDR,I2C_ENABLE_SR);
        I2C_MasterReceiveData(&I2C1Handle,&len,1,SLAVE_ADDR,I2C_ENABLE_SR);

        commandcode = 0x52;

        I2C_MasterSendData(&I2C1Handle,&commandcode,1,SLAVE_ADDR,I2C_ENABLE_SR);
        I2C_MasterReceiveData(&I2C1Handle,rcv_buf,len,SLAVE_ADDR,I2C_DISABLE_SR);

        rcv_buf[len+1] = '\0';

        printf("Data : %s",rcv_buf);
    }
}
