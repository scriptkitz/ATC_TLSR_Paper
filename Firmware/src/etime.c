#include <stdint.h>
#include "tl_common.h"
#include "drivers.h"
#include "stack/ble/ble.h"
#include "drivers/8258/flash.h"
#include "etime.h"
#include "main.h"

RAM uint16_t time_trime = 5000;// The higher the number the slower the time runs!, -32,768 to 32,767 
RAM uint32_t one_second_trimmed = CLOCK_16M_SYS_TIMER_CLK_1S;
RAM uint32_t current_unix_time;
RAM uint32_t last_clock_increase;
RAM uint32_t last_reached_period[10] = {0};
RAM uint8_t has_ever_reached[10] = {0};

_attribute_ram_code_ void init_time(void)
{
    one_second_trimmed += time_trime;
    current_unix_time = 0;
}

_attribute_ram_code_ void handler_time(void)
{
    if (clock_time() - last_clock_increase >= one_second_trimmed)
    {
        last_clock_increase += one_second_trimmed;
        current_unix_time++;
    }
}

_attribute_ram_code_ uint8_t time_reached_period(timer_channel ch, uint32_t seconds)
{
    if (!has_ever_reached[ch])
    {
        has_ever_reached[ch] = 1;
        return 1;
    }
    if (current_unix_time - last_reached_period[ch] >= seconds)
    {
        last_reached_period[ch] = current_unix_time;
        return 1;
    }
    return 0;
}

_attribute_ram_code_ void set_time(uint32_t time_now)
{
    current_unix_time = time_now;
}

_attribute_ram_code_ uint32_t get_time(void)
{
    return current_unix_time;
}

#define SECONDS_PER_DAY 86400
#define SECONDS_PER_HOUR 3600
#define SECONDS_PER_MINUTE 60

_attribute_ram_code_ bool is_leap_year(int year)
{
    return (year % 4 == 0 && year % 100 != 0) || ((year % 400) == 0);
}

_attribute_ram_code_ uint32_t get_from_dt(const struct date_time *dt)
{
    uint32_t days_since_epoch = 0;
    for (int y = 1970; y < dt->tm_year; y++)
    {
        days_since_epoch += (is_leap_year(y) ? 366 : 365);
    }
    int days_in_month[] = {31, is_leap_year(dt->tm_year)?29:28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    for (int m = 1; m < dt->tm_month; m++)
    {
        days_since_epoch += days_in_month[m-1];
    }
    days_since_epoch += dt->tm_day - 1;

    return days_since_epoch * SECONDS_PER_DAY + dt->tm_hour * SECONDS_PER_HOUR + dt->tm_min * SECONDS_PER_MINUTE;
}

_attribute_ram_code_ struct date_time* get_from_ts(uint32_t timestamp, struct date_time* dt)
{
    uint32_t days_since_epoch = 0, reminder = 0;
    int year, month;

    days_since_epoch = timestamp / SECONDS_PER_DAY;
    reminder = timestamp % SECONDS_PER_DAY;
    dt->tm_hour = reminder / SECONDS_PER_HOUR;
    reminder = reminder % SECONDS_PER_HOUR;
    dt->tm_min = reminder / SECONDS_PER_MINUTE;
    dt->tm_sec = reminder % SECONDS_PER_MINUTE;

    year = 1970;
    while (days_since_epoch >= 365)
    {
        days_since_epoch -= (is_leap_year(year) ? 366 : 365);
        year++;
    }
    dt->tm_year = year;

    month = 1;
    int month_lengths[12] = {31, is_leap_year(year) ? 29 : 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    while (days_since_epoch >= month_lengths[month - 1])
    {
        days_since_epoch -= month_lengths[month - 1];
        month++;
    }
    dt->tm_month = month;
    dt->tm_day = days_since_epoch + 1;
    return dt;
}

uint8_t get_week(const struct date_time *dt)
{
    int year = dt->tm_year;
    int month = dt->tm_month;
    int day = dt->tm_day;
    if (month == 1 || month == 2) {
        month += 12;
        year--;
    }
    int century = year / 100;
    year %= 100;
    int week = year + year / 4 + century / 4 - 2 * century + 26 * (month + 1) / 10 + day - 1;
    week = (week % 7 + 7) % 7;
    return week;
}