/*
 * lcd.h
 *
 *  Created on: Mar 6, 2023
 *      Author: nemanja
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_

#include <stm32f446xx.h>

/* BSP Exposed APIs */
void LCD_Init(void);
void LCD_SendCommand(uint8_t command);
void LCD_PrintChar(uint8_t data);
void LCD_DisplayClear(void);
void LCD_DisplayReturnHome(void);
void LCD_PrintString(char*);
void LCD_SetCursor(uint8_t row, uint8_t column);

/* Application configurable items */
#define LCD_GPIO_PORT   GPIOC
#define LCD_GPIO_RS     GPIO_PIN_NO_0
#define LCD_GPIO_RW     GPIO_PIN_NO_1
#define LCD_GPIO_EN     GPIO_PIN_NO_2
#define LCD_GPIO_D4     GPIO_PIN_NO_3
#define LCD_GPIO_D5     GPIO_PIN_NO_4
#define LCD_GPIO_D6     GPIO_PIN_NO_5
#define LCD_GPIO_D7     GPIO_PIN_NO_6

/* LCD Commands */
#define LCD_CMD_4DL_2N_5X8F         0x28
#define LCD_CMD_DON_CURON           0x0E
#define LCD_CMD_INCADD              0x06
#define LCD_CMD_DIS_CLEAR           0X01
#define LCD_CMD_DIS_RETURN_HOME     0x02

#endif /* INC_LCD_H_ */
