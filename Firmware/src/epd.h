#pragma once
#include "etime.h"

#define EPD_MAX_HEIGHT 128
#define EPD_MAX_WIDTH 250
// 上面的宽高如果过大，会导致编译报错.text过大了，可以研究一下.retention_data段大小问题
#define epd_buffer_size ((EPD_MAX_HEIGHT/8) * EPD_MAX_WIDTH)

void set_EPD_model(uint8_t model_nr);
void set_EPD_scene(uint8_t scene);
void set_EPD_wait_flush();

void init_epd(void);
uint8_t EPD_read_temp(void);
void EPD_Display(unsigned char *image, unsigned char * red_image, int size, uint8_t full_or_partial);
void epd_display_tiff(uint8_t *pData, int iSize);
void epd_set_sleep(void);
uint8_t epd_state_handler(void);
void epd_display_char(uint8_t data);
void epd_clear(void);

void update_time_scene(uint32_t _time, uint16_t battery_mv, int16_t temperature, void (*scene)(const struct date_time*, uint16_t, int16_t,  uint8_t));
void epd_update(uint32_t _time, uint16_t battery_mv, int16_t temperature);

void epd_display(const struct date_time *time_is, uint16_t battery_mv, int16_t temperature, uint8_t full_or_partial);
void epd_display_time_with_date(const struct date_time *_time, uint16_t battery_mv, int16_t temperature, uint8_t full_or_partial);
