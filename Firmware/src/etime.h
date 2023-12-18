#pragma once

typedef enum
{
    Timer_CH_0 = 0,
    Timer_CH_1,
    Timer_CH_2,
    Timer_CH_3,
    Timer_CH_4,
    Timer_CH_5,
    Timer_CH_6,
    Timer_CH_7,
    Timer_CH_8,
    Timer_CH_9,
} timer_channel;

struct date_time
{
  short	tm_year;
  short	tm_month;
  short	tm_day;
  short	tm_hour;
  short	tm_min;
  short	tm_sec;
};

void init_time(void);
void handler_time(void);
uint8_t time_reached_period(timer_channel ch, uint32_t seconds);
void set_time(uint32_t time_now);
uint32_t get_time(void);
uint8_t get_week(const struct date_time *dt);
uint32_t get_from_dt(const struct date_time *dt);
struct date_time* get_from_ts(uint32_t timestamp, struct date_time* dt);