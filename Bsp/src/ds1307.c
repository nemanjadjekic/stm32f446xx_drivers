/*
 * ds1307.c
 *
 *  Created on: Feb 1, 2023
 *      Author: nemanja
 */


#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <ds1307.h>

static void DS1307_I2C_Config(void);
static void DS1307_I2C_PinConfig(void);
static void DS1307_Write(uint8_t Value, uint8_t RegAddr);
static uint8_t DS1307_Read(uint8_t RegAddr);
static uint8_t BcdToBinary(uint8_t value);
static uint8_t BinaryToBcd(uint8_t value);

I2C_Handle_t g_DS1307_I2C_Handle;


/*****************************************************************
 * @fn          - DS1307_Init
 *
 * @brief       - This function initialize necessary I2C peripherals
 *                needed to communicate with DS1307
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
uint8_t DS1307_Init(void)
{
    // Initialize I2C Pins
    DS1307_I2C_PinConfig();

    // Initialize I2C Peripherals
    DS1307_I2C_Config();

    // Enable I2C Peripherals
    I2C_PeripheralControl(DS1307_I2C, ENABLE);

    // Make clock halt = 0
    DS1307_Write(0x00, DS1307_ADDR_SECONDS);

    // Read back clock halt bit
    uint8_t clock_state = DS1307_Read(DS1307_ADDR_SECONDS);

    return ((clock_state >> 7 ) & 0x1);
}


/*****************************************************************
 * @fn          - DS1307_SetCurrentTime
 *
 * @brief       - This function sets current time
 *
 * @param[in]   - Addres of RTC Time structure
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void DS1307_SetCurrentTime(RTC_Time_t *RTC_Time)
{
    uint8_t seconds = BinaryToBcd(RTC_Time->seconds);
    seconds &= ~(1 << 7);
    DS1307_Write(seconds, DS1307_ADDR_SECONDS);

    uint8_t minutes = BinaryToBcd(RTC_Time->minutes);
    DS1307_Write(minutes, DS1307_ADDR_MINUTES);

    uint8_t hours = BinaryToBcd(RTC_Time->hours);

    if(RTC_Time->time_format == TIME_FORMAT_24HRS) {
        hours &= ~(1 << 6);
    } else {
        hours |= (1 << 6);
        if(RTC_Time->time_format == TIME_FORMAT_12HRS_PM) {
            hours |= (1 << 5);
        } else {
            hours &= ~(1 << 5);
        }
    }

    DS1307_Write(hours, DS1307_ADDR_HOURS);
}


/*****************************************************************
 * @fn          - DS1307_GetCurrentTime
 *
 * @brief       - This function gets time data and stores it
 *                into RTC Time structure
 *
 * @param[in]   - Addres of RTC Time structure
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void DS1307_GetCurrentTime(RTC_Time_t *RTC_Time)
{
    uint8_t seconds = DS1307_Read(DS1307_ADDR_SECONDS);
    seconds &= ~(1 << 7);
    RTC_Time->seconds = BcdToBinary(seconds);

    uint8_t minutes = DS1307_Read(DS1307_ADDR_MINUTES);
    RTC_Time->minutes = BcdToBinary(minutes);

    uint8_t hours = DS1307_Read(DS1307_ADDR_HOURS);

    if(hours & (1 << 6)) {
        // 12 hour format
        RTC_Time->time_format = !((hours & (1 << 5)) == 0);
        hours &= ~(0x03 << 5); // Clear 5th and 6th bits
    } else {
        // 24 hour format
        RTC_Time->time_format = TIME_FORMAT_24HRS;
    }

    RTC_Time->hours = BcdToBinary(hours);
}


/*****************************************************************
 * @fn          - DS1307_SetCurrentDate
 *
 * @brief       - This function sets current date
 *
 * @param[in]   - Addres of RTC Date structure
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void DS1307_SetCurrentDate(RTC_Date_t *RTC_Date)
{
    DS1307_Write(BinaryToBcd(RTC_Date->date), DS1307_ADDR_DATE);
    DS1307_Write(BinaryToBcd(RTC_Date->month), DS1307_ADDR_MONTH);
    DS1307_Write(BinaryToBcd(RTC_Date->year), DS1307_ADDR_YEAR);
    DS1307_Write(BinaryToBcd(RTC_Date->day), DS1307_ADDR_DAY);
}


/*****************************************************************
 * @fn          - DS1307_GetCurrentDate
 *
 * @brief       - This function gets date data and stores it
 *                into RTC Date structure
 *
 * @param[in]   - Addres of RTC Date structure
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void DS1307_GetCurrentDate(RTC_Date_t *RTC_Date)
{
    RTC_Date->day = BcdToBinary(DS1307_Read(DS1307_ADDR_DAY));
    RTC_Date->date = BcdToBinary(DS1307_Read(DS1307_ADDR_DATE));
    RTC_Date->month = BcdToBinary(DS1307_Read(DS1307_ADDR_MONTH));
    RTC_Date->year = BcdToBinary(DS1307_Read(DS1307_ADDR_YEAR));
}


/*****************************************************************
 * @fn          - DS1307_I2C_PinConfig
 *
 * @brief       - This function configures I2C pins
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
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


/*****************************************************************
 * @fn          - DS1307_I2C_Config
 *
 * @brief       - This function configures I2C settings
 *                (clocks/speed/mode)
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
static void DS1307_I2C_Config(void)
{
    g_DS1307_I2C_Handle.pI2Cx = DS1307_I2C;
    g_DS1307_I2C_Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
    g_DS1307_I2C_Handle.I2C_Config.I2C_SCLSpeed = DS1307_I2C_SPEED;
    I2C_Init(&g_DS1307_I2C_Handle);
}


/*****************************************************************
 * @fn          - DS1307_Write
 *
 * @brief       - This function writes to DS1307 memory
 *                utilizing I2C communication
 *
 * @param[in]   - Value that should be written
 * @param[in]   - Address where value should be written in memory
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
static void DS1307_Write(uint8_t Value, uint8_t RegAddr)
{
    uint8_t tx[2];
    tx[0] = RegAddr;
    tx[1] = Value;
    I2C_MasterSendData(&g_DS1307_I2C_Handle, tx, 2, DS1307_I2C_ADDRESS, 0);
}


/*****************************************************************
 * @fn          - DS1307_Read
 *
 * @brief       - This function reads data from DS1307 memory
 *                utilizing I2C communication
 *
 * @param[in]   - Address from which data should be read
 *
 * @return      - Data read from given address
 *
 * @Note        - None
 *
 *****************************************************************/
static uint8_t DS1307_Read(uint8_t RegAddr)
{
    uint8_t data;
    I2C_MasterSendData(&g_DS1307_I2C_Handle, &RegAddr, 1, DS1307_I2C_ADDRESS, 0);
    I2C_MasterReceiveData(&g_DS1307_I2C_Handle, &data, 1, DS1307_I2C_ADDRESS, 0);

    return data;
}


/*****************************************************************
 * @fn          - BinaryToBcd
 *
 * @brief       - This function converts binary (uint8_t) data
 *                to binary-coded decimal data format
 *
 * @param[in]   - Value that should be converted
 *
 * @return      - Binary-coded decimal
 *
 * @Note        - None
 *
 *****************************************************************/
static uint8_t BinaryToBcd(uint8_t value)
{
    uint8_t high_nibble;
    uint8_t low_nibble;
    uint8_t bcd;

    bcd = value;
    if(value >= 10)
    {
        high_nibble = value / 10;
        low_nibble = value % 10;
        bcd = (high_nibble << 4) | low_nibble;
    }

    return bcd;
}


/*****************************************************************
 * @fn          - BcdToBinary
 *
 * @brief       - This function converts binary-coded decimal to
 *                binary (uint8_t) format
 *
 * @param[in]   - Value that should be converted
 *
 * @return      - Binary (uint8_t)
 *
 * @Note        - None
 *
 *****************************************************************/
static uint8_t BcdToBinary(uint8_t value)
{
    uint8_t high_nibble = (uint8_t)((value >> 4) * 10);
    uint8_t low_nibble = value & (uint8_t)0x0F;

    return (high_nibble + low_nibble);
}
