/*
 * 017_RTC_LCD.c
 *
 *  Created on: Feb 16, 2023
 *      Author: nemanja
 */

#include <stdio.h>
#include <ds1307.h>
#include <lcd.h>

#define SYSTICK_TIMER_CLK   1600000UL

/* Helper functions */
char* GetDayOfWeek(uint8_t i);
void NumberToString(uint8_t num, char *buf);
char* TimeToString(RTC_Time_t *RTC_Time);
char* DateToString(RTC_Date_t *RTC_Date);
void InitSystickTimer(uint32_t tick_hz);

static void Delay_ms(uint32_t value);


int main()
{
    RTC_Time_t current_time;
    RTC_Date_t current_date;

    /* LCD Settings */
    LCD_Init();

    LCD_PrintString("RTC Test...");

    Delay_ms(2000);

    LCD_DisplayClear();

    LCD_DisplayReturnHome();

    if(DS1307_Init())
    {
        printf("RTC initialization has failed\n");
        while(1);
    }

    InitSystickTimer(1);

    current_date.day = THURSDAY;
    current_date.date = 16;
    current_date.month = 2;
    current_date.year = 23;

    current_time.hours = 23;
    current_time.minutes = 59;
    current_time.seconds = 55;
    current_time.time_format = TIME_FORMAT_24HRS;

    DS1307_SetCurrentDate(&current_date);
    DS1307_SetCurrentTime(&current_time);

    DS1307_GetCurrentDate(&current_date);
    DS1307_GetCurrentTime(&current_time);

    char *AM_PM;
    if(current_time.time_format != TIME_FORMAT_24HRS)
    {
        AM_PM = (current_time.time_format) ? "PM" : "AM";

        LCD_PrintString(TimeToString(&current_time));
        LCD_PrintString(AM_PM);
    }
    else
    {
        LCD_PrintString(TimeToString(&current_time));
    }

    LCD_SetCursor(2, 1);
    LCD_PrintString(DateToString(&current_date));

    while(1);

    return 0;
}

/* Helper functions */
char* GetDayOfWeek(uint8_t i)
{
    char* days[] = {"Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"};
    return days[i - 1];
}

void NumberToString(uint8_t num, char *buf)
{
    if(num < 10)
    {
        buf[0] = '0';
        buf[1] = num + 48;

    }
    else if(num >= 10 && num < 99)
    {
        buf[0] = (num / 10) + 48;
        buf[1] = (num % 10) + 48;
    }
}

char* TimeToString(RTC_Time_t *RTC_Time)
{
    static char buf[9];

    buf[2] = ':';
    buf[5] = ':';

    NumberToString(RTC_Time->hours, buf);
    NumberToString(RTC_Time->minutes, &buf[3]);
    NumberToString(RTC_Time->seconds, &buf[6]);

    buf[8] = '\0';

    return buf;
}

char* DateToString(RTC_Date_t *RTC_Date)
{
    static char buf[9];

    buf[2] = '/';
    buf[5] = '/';

    NumberToString(RTC_Date->date, buf);
    NumberToString(RTC_Date->month, &buf[3]);
    NumberToString(RTC_Date->year, &buf[6]);

    buf[8] = '\0';

    return buf;
}

void InitSystickTimer(uint32_t tick_hz)
{
    uint32_t *pSCSR = (uint32_t*)0xE000E010;
    uint32_t *pSRVR = (uint32_t*)0xE000E014;

    /* Calculation of reload value */
    uint32_t count_value = (SYSTICK_TIMER_CLK / tick_hz) - 1;

    /* Clear the value of SVR */
    *pSRVR &= ~(0x00FFFFFFFF);

    /* Load the value into SVR */
    *pSRVR |= count_value;

    /* Settings: Enable SysTick exception request */
    *pSCSR |= (1 << 1);

    /* Settings: Indicate processor clock source */
    *pSCSR |= (1 << 2);

    /* Settings: Enable SysTick counter */
    *pSCSR |= (1 << 0);
}

void SysTick_Handler(void)
{
    RTC_Time_t current_time;
    RTC_Date_t current_date;

    DS1307_GetCurrentDate(&current_date);

    char *AM_PM;
    if(current_time.time_format != TIME_FORMAT_24HRS)
    {
        AM_PM = (current_time.time_format) ? "PM" : "AM";

        LCD_SetCursor(1, 1);
        LCD_PrintString(TimeToString(&current_time));
        LCD_PrintString(AM_PM);
    }
    else
    {
        LCD_SetCursor(1, 1);
        LCD_PrintString(TimeToString(&current_time));
    }

    DS1307_GetCurrentTime(&current_time);

    LCD_SetCursor(2, 1);
    LCD_PrintString(DateToString(&current_date));
    LCD_PrintChar('<');
    LCD_PrintString(GetDayOfWeek(current_date.day));
    LCD_PrintChar('>');
}

static void Delay_ms(uint32_t value)
{
    for(uint32_t i = 0 ; i < (value * 1000); i++);
}
