/*
 * lcd.c
 *
 *  Created on: Feb 1, 2023
 *      Author: nemanja
 */


#include <lcd.h>

static void Write4Bits(uint8_t value);
static void LCD_Enable(void);
static void Delay_ms(uint8_t value);
static void Delay_us(uint8_t value);


/*****************************************************************
 * @fn          - LCD_Init
 *
 * @brief       - This function initializes all necessary
 *                peripherals in order to communicate with LCD
 *
 * @param[in]   - None
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void LCD_Init(void)
{
    /* Configure the GPIO pins used for LCD connections */
    GPIO_Handle_t LCD_Signal;

    LCD_Signal.pGPIOx = LCD_GPIO_PORT;
    LCD_Signal.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    LCD_Signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_RS;
    LCD_Signal.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    LCD_Signal.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;
    LCD_Signal.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GPIO_Init(&LCD_Signal);

    LCD_Signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_RW;
    GPIO_Init(&LCD_Signal);

    LCD_Signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_EN;
    GPIO_Init(&LCD_Signal);

    LCD_Signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D4;
    GPIO_Init(&LCD_Signal);

    LCD_Signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D5;
    GPIO_Init(&LCD_Signal);

    LCD_Signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D6;
    GPIO_Init(&LCD_Signal);

    LCD_Signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D7;
    GPIO_Init(&LCD_Signal);


    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_RESET);
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, GPIO_PIN_RESET);
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, GPIO_PIN_RESET);
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, GPIO_PIN_RESET);
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, GPIO_PIN_RESET);

    /* LCD Initialization */
    Delay_ms(40);

    /* RS = 0; LCD Command */
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);

    /* Read and Write; Writing to LCD */
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

    Write4Bits(0x03);

    Delay_ms(5);

    Delay_us(150);

    Write4Bits(0x03);
    Write4Bits(0x02);

    /* Set command */
    LCD_SendCommand(LCD_CMD_4DL_2N_5X8F);

    /* Display and cursor ON */
    LCD_SendCommand(LCD_CMD_DON_CURON);

    LCD_DisplayClear();

    /* Entry mode set */
    LCD_SendCommand(LCD_CMD_INCADD);
}


/*****************************************************************
 * @fn          - LCD_SendCommand
 *
 * @brief       - This function sends commands for LCD managing
 *
 * @param[in]   - Command
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void LCD_SendCommand(uint8_t command)
{
    /* RS = 0 for LCD Command */
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);

    /* RW = 0 for Write */
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

    Write4Bits(command >> 4);
    Write4Bits(command & 0x0F);
}


/*****************************************************************
 * @fn          - LCD_PrintChar
 *
 * @brief       - This function sends characters to LCD
 *
 * @param[in]   - Data
 *
 * @return      - None
 *
 * @Note        - In this case first 4 bits are used for data
 *                transmission. First higher nibble of the data
 *                will be sent via D4, D5, D6 and D7 data lines.
 *                Then lower nibble of the data will be sent via
 *                D4, D5, D6 and D7 data lines.
 *
 *****************************************************************/
void LCD_PrintChar(uint8_t data)
{
    /* RS = 1 for User Data */
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_SET);

    /* RW = 0 for Write */
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

    Write4Bits(data >> 4);   /* Higher nibble */
    Write4Bits(data & 0x0F); /* Lower nibble  */
}


/*****************************************************************
 * @fn          - LCD_ClearDisplay
 *
 * @brief       - This function clears LCD
 *
 * @param[in]   - None
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void LCD_DisplayClear(void)
{
    LCD_SendCommand(LCD_CMD_DIS_CLEAR);

    /* Data sheet; Page 24
     * Display clear command execution time is around 2 ms
     */
    Delay_ms(5);
}


/*****************************************************************
 * @fn          - LCD_DisplayReturnHome
 *
 * @brief       - This function returns cursor to home position
 *
 * @param[in]   - None
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void LCD_DisplayReturnHome(void)
{
    LCD_SendCommand(LCD_CMD_DIS_RETURN_HOME);

    /* Data sheet; Page 24
     * Display clear command execution time is around 2 ms
     */
    Delay_ms(2);
}


/*****************************************************************
 * @fn          - LCD_PrintString
 *
 * @brief       - This function sends string to LCD
 *
 * @param[in]   - String
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void LCD_PrintString(char* message)
{
    do
    {
        LCD_PrintChar((uint8_t)* message++);
    }
    while(*message != '\0');
}


/*****************************************************************
 * @fn          - LCD_SetCursor
 *
 * @brief       - This function sets cursor position
 *
 * @param[in]   - Row
 * @param[in]   - Column
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void LCD_SetCursor(uint8_t row, uint8_t column)
{
    column--;
    switch(row)
    {
        case 1:
            /* Set cursor to 1st row address and add index */
            LCD_SendCommand((column |= 0x80));
            break;
        case 2:
            /* Set cursor to 2nd row address and add index */
            LCD_SendCommand((column |= 0xC0));
            break;

        default:
            break;
    }
}


/*****************************************************************
 * @fn          - Write4Bits
 *
 * @brief       - This function writes 4 bits of data/command
 *                on D4, D5, D6 and D7 data lines
 *
 * @param[in]   - None
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
static void Write4Bits(uint8_t value)
{
    GPIO_WriteToOutputPin(LCD_GPIO_PORT,LCD_GPIO_D4, ((value >> 0) & 0x1) );
    GPIO_WriteToOutputPin(LCD_GPIO_PORT,LCD_GPIO_D5, ((value >> 1) & 0x1) );
    GPIO_WriteToOutputPin(LCD_GPIO_PORT,LCD_GPIO_D6, ((value >> 2) & 0x1) );
    GPIO_WriteToOutputPin(LCD_GPIO_PORT,LCD_GPIO_D7, ((value >> 3) & 0x1) );

    LCD_Enable();
}


/*****************************************************************
 * @fn          - LCD_Enable
 *
 * @brief       - This function enables LCD
 *
 * @param[in]   - None
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
static void LCD_Enable(void)
{
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_SET);
    Delay_us(10);

    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_RESET);
    /* Execution time > 37 microseconds */
    Delay_us(100);
}


/*****************************************************************
 * @fn          - Delay_ms
 *
 * @brief       - This function sets delay for code execution
 *
 * @param[in]   - Value in milliseconds
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
static void Delay_ms(uint8_t value)
{
    for(uint32_t i = 0 ; i < (value * 1000); i++);
}


/*****************************************************************
 * @fn          - Delay_us
 *
 * @brief       - This function sets delay for code execution
 *
 * @param[in]   - Value in microseconds
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
static void Delay_us(uint8_t value)
{
    for(uint32_t i = 0 ; i < (value * 1); i++);
}
