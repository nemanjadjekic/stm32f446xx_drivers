/*
 * ds1307.h
 *
 *  Created on: Feb 1, 2023
 *      Author: nemanja
 */

#ifndef INC_DS1307_H_
#define INC_DS1307_H_

#include <stm32f446xx.h>


/* Application configurable items */
#define DS1307_I2C              I2C1
#define DS1307_I2C_GPIO_PORT    GPIOB
#define DS1307_I2C_SDA_PIN      GPIO_PIN_NO_7
#define DS1307_I2C_SCL_PIN      GPIO_PIN_NO_6
#define DS1307_I2C_SPEED        I2C_SCL_SPEED_SM
#define DS1307_I2C_PUPD         GPIO_PIN_PU


/* Register addresses */
#define DS1307_ADDR_SECONDS     0x00
#define DS1307_ADDR_MINUTES     0x01
#define DS1307_ADDR_HOURS       0x02
#define DS1307_ADDR_DAY         0x03
#define DS1307_ADDR_DATE        0x04
#define DS1307_ADDR_MONTH       0x05
#define DS1307_ADDR_YEAR        0x06

#define TIME_FORMAT_12HRS_AM    0
#define TIME_FORMAT_12HRS_PM    1
#define TIME_FORMAT_24HRS       2

#define DS1307_I2C_ADDRESS      0x68

#define MONDAY      1
#define TUESDAY     2
#define WEDNESDAY   3
#define THURSDAY    4
#define FRIDAY      5
#define SATURDAY    6
#define SUNDAY      7


typedef struct
{
    uint8_t date;
    uint8_t month;
    uint8_t year;
    uint8_t day;

}RTC_Date_t;

typedef struct
{
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hours;
    uint8_t time_format;

}RTC_Time_t;


/* Function prototypes */
uint8_t DS1307_Init(void);

void DS1307_SetCurrentTime(RTC_Time_t *);
void DS1307_GetCurrentTime(RTC_Time_t *);

void DS1307_SetCurrentDate(RTC_Date_t *);
void DS1307_GetCurrentDate(RTC_Date_t *);

#endif /* INC_DS1307_H_ */
