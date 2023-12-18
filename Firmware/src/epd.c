#include <stdint.h>
#include "tl_common.h"
#include "main.h"
#include "epd.h"
#include "epd_spi.h"


#include "epd_bw_213_ice.h"

#include "drivers.h"
#include "stack/ble/ble.h"

#include "battery.h"

#include "OneBitDisplay.h"
#include "TIFF_G4.h"
extern const uint8_t ucMirror[];
#include "font_60.h"
#include "font16.h"
#include "font30.h"
#include "font9.h"
#include "font16zh.h"

RAM uint8_t epd_model = 0; // 0 = Undetected, 1 = BW213, 2 = BWR213, 3 = BWR154, 4 = BW213ICE
const char *epd_model_string[] = {"NC", "BW213", "BWR213", "BWR154", "213ICE"};
RAM uint8_t epd_update_state = 0;

RAM uint8_t epd_scene = 2;
RAM uint8_t epd_wait_update = 0;

RAM uint8_t last_hour = 0xFF;
RAM uint8_t last_minute = 0xFF;

const char *BLE_conn_string[] = {"", "B"};
RAM uint8_t epd_temperature_is_read = 0;
RAM uint8_t epd_temperature = 0;

RAM uint8_t epd_buffer[epd_buffer_size];
RAM uint8_t epd_temp[epd_buffer_size]; // for OneBitDisplay to draw into
OBDISP obd;                        // virtual display structure
TIFFIMAGE tiff;

// With this we can force a display if it wasnt detected correctly
void set_EPD_model(uint8_t model_nr)
{
    epd_model = model_nr;
}

// With this we can force a display if it wasnt detected correctly
void set_EPD_scene(uint8_t scene)
{
    epd_scene = scene;
    set_EPD_wait_flush();
}

void set_EPD_wait_flush() {
    epd_wait_update = 1;
}



// Here we detect what E-Paper display is connected
_attribute_ram_code_ void EPD_detect_model(void)
{
    epd_model = 4;

}

_attribute_ram_code_ uint8_t EPD_read_temp(void)
{
    if (epd_temperature_is_read)
        return epd_temperature;

    if (!epd_model)
        EPD_detect_model();

    EPD_init();
    // system power
    EPD_POWER_ON();
    WaitMs(5);
    // Reset the EPD driver IC
    gpio_write(EPD_RESET, 0);
    WaitMs(10);
    gpio_write(EPD_RESET, 1);
    WaitMs(10);

    epd_temperature = EPD_BW_213_ice_read_temp();

    EPD_POWER_OFF();

    epd_temperature_is_read = 1;

    return epd_temperature;
}

_attribute_ram_code_ void EPD_Display(unsigned char *image, unsigned char *red_image, int size, uint8_t full_or_partial)
{
    if (!epd_model)
        EPD_detect_model();

    EPD_init();
    // system power
    EPD_POWER_ON();
    WaitMs(5);
    // Reset the EPD driver IC
    gpio_write(EPD_RESET, 0);
    WaitMs(10);
    gpio_write(EPD_RESET, 1);
    WaitMs(10);


    epd_temperature = EPD_BW_213_ice_Display(image, size, full_or_partial);

    epd_temperature_is_read = 1;
    epd_update_state = 1;
}

_attribute_ram_code_ void epd_set_sleep(void)
{
    if (!epd_model)
        EPD_detect_model();


    EPD_BW_213_ice_set_sleep();

    EPD_POWER_OFF();
    epd_update_state = 0;
}

_attribute_ram_code_ uint8_t epd_state_handler(void)
{
    switch (epd_update_state)
    {
    case 0:
        // Nothing todo
        break;
    case 1: // check if refresh is done and sleep epd if so
        if (epd_model == 1)
        {
            if (!EPD_IS_BUSY())
                epd_set_sleep();
        }
        else
        {
            if (EPD_IS_BUSY())
                epd_set_sleep();
        }
        break;
    }
    return epd_update_state;
}

_attribute_ram_code_ void FixBuffer(uint8_t *pSrc, uint8_t *pDst, uint16_t width, uint16_t height)
{
    int x, y;
    uint8_t *s, *d;
    for (y = 0; y < (height / 8); y++)
    { // byte rows
        d = &pDst[y];
        s = &pSrc[y * width];
        for (x = 0; x < width; x++)
        {
            d[x * (height / 8)] = ~ucMirror[s[width - 1 - x]]; // invert and flip
        }                                                      // for x
    }                                                          // for y
}

_attribute_ram_code_ void TIFFDraw(TIFFDRAW *pDraw)
{
    uint8_t uc = 0, ucSrcMask, ucDstMask, *s, *d;
    int x, y;

    s = pDraw->pPixels;
    y = pDraw->y;                          // current line
    d = &epd_buffer[(249 * 16) + (y / 8)]; // rotated 90 deg clockwise
    ucDstMask = 0x80 >> (y & 7);           // destination mask
    ucSrcMask = 0;                         // src mask
    for (x = 0; x < pDraw->iWidth; x++)
    {
        // Slower to draw this way, but it allows us to use a single buffer
        // instead of drawing and then converting the pixels to be the EPD format
        if (ucSrcMask == 0)
        { // load next source byte
            ucSrcMask = 0x80;
            uc = *s++;
        }
        if (!(uc & ucSrcMask))
        { // black pixel
            d[-(x * 16)] &= ~ucDstMask;
        }
        ucSrcMask >>= 1;
    }
}

_attribute_ram_code_ void epd_display_tiff(uint8_t *pData, int iSize)
{
    // test G4 decoder
    epd_clear();
    TIFF_openRAW(&tiff, 250, 122, BITDIR_MSB_FIRST, pData, iSize, TIFFDraw);
    TIFF_setDrawParameters(&tiff, 65536, TIFF_PIXEL_1BPP, 0, 0, 250, 122, NULL);
    TIFF_decode(&tiff);
    TIFF_close(&tiff);
    EPD_Display(epd_buffer, NULL, epd_buffer_size, 1);
}

_attribute_ram_code_ void epd_get_display_size(uint16_t *width, uint16_t *height)
{
    if (!epd_model) EPD_detect_model();

    *width = 250;
    *height = 128; // 122 real pixel, but needed to have a full byte
    switch(epd_model)
    {
        case 1: break;
        case 2: break;
        case 3:
            *width = 200;
            *height = 200;
            break;
        case 4:
            *height = 128;
            break;
    }
}

extern uint8_t mac_public[6];
_attribute_ram_code_ void epd_display(const struct date_time *dt, uint16_t battery_mv, int16_t temperature, uint8_t full_or_partial)
{
    if (epd_update_state)
        return;

    epd_clear();
    uint16_t resolution_w = 0;
    uint16_t resolution_h = 0;
    epd_get_display_size(&resolution_w, &resolution_h);
    obdCreateVirtualDisplay(&obd, resolution_w, resolution_h, epd_temp);
    obdFill(&obd, 0, 0); // fill with white

    char buff[100];
    sprintf(buff, "ESL_%02X%02X%02X %s", mac_public[2], mac_public[1], mac_public[0], epd_model_string[epd_model]);
    obdWriteStringCustom(&obd, (GFXfont *)&Dialog_plain_16, 1, 17, (char *)buff, 1);
    sprintf(buff, "%s", BLE_conn_string[ble_get_connected()]);
    obdWriteStringCustom(&obd, (GFXfont *)&Dialog_plain_16, 232, 20, (char *)buff, 1);
    sprintf(buff, "%02d:%02d", dt->tm_hour, dt->tm_min);
    obdWriteStringCustom(&obd, (GFXfont *)&DSEG14_Classic_Mini_Regular_40, 50, 65, (char *)buff, 1);
    sprintf(buff, "%d'C", EPD_read_temp());
    obdWriteStringCustom(&obd, (GFXfont *)&Special_Elite_Regular_30, 10, 95, (char *)buff, 1);
    sprintf(buff, "Battery %dmV %d%%", battery_mv, get_battery_level(battery_mv));
    obdWriteStringCustom(&obd, (GFXfont *)&Dialog_plain_16, 10, 120, (char *)buff, 1);

    sprintf(buff, "%s", "oh...my god!!! :p");
    obdWriteStringCustom(&obd, (GFXfont *)&Orbitron_Medium_9, 80, 100, (char *)buff, 1);

    FixBuffer(epd_temp, epd_buffer, resolution_w, resolution_h);
    EPD_Display(epd_buffer, NULL, resolution_w * resolution_h / 8, full_or_partial);
}

_attribute_ram_code_ void epd_display_char(uint8_t data)
{
    int i;
    for (i = 0; i < epd_buffer_size; i++)
    {
        epd_buffer[i] = data;
    }
    EPD_Display(epd_buffer, NULL, epd_buffer_size, 1);
}

_attribute_ram_code_ void epd_clear(void)
{
    memset(epd_buffer, 0x00, epd_buffer_size);
    memset(epd_temp, 0x00, epd_buffer_size);
}

_attribute_ram_code_ void update_time_scene(uint32_t _time, uint16_t battery_mv, int16_t temperature, void (*scene)(const struct date_time*, uint16_t, int16_t,  uint8_t)) {
    // default scene: show default time, battery, ble address, temperature
    if (epd_update_state)
        return;

    if (!epd_model)
    {
        EPD_detect_model(); 
    }
    struct date_time dt;
    get_from_ts(_time, &dt);

    if (epd_wait_update) {
        scene(&dt, battery_mv, temperature, 1);
        epd_wait_update = 0;
    }
    
    else if (dt.tm_min != last_minute)
    {
        last_minute = dt.tm_min;
        if (dt.tm_hour != last_hour)
        {
            last_hour = dt.tm_hour;
            scene(&dt, battery_mv, temperature, 1);
        }
        else
        {
            scene(&dt, battery_mv, temperature, 0);
        }
    }
}

_attribute_ram_code_ void epd_update(uint32_t _time, uint16_t battery_mv, int16_t temperature) {
    switch(epd_scene) {
        case 1:
            update_time_scene(_time, battery_mv, temperature, epd_display);
            break;
        case 2:
            update_time_scene(_time, battery_mv, temperature, epd_display_time_with_date);
            break;
        default:
            break;
    }
}

_attribute_ram_code_ void epd_display_time_with_date(const struct date_time *dt, uint16_t battery_mv, int16_t temperature, uint8_t full_or_partial) {
    uint16_t battery_level;

    epd_clear();
    
    uint16_t _w = 0;
    uint16_t _h = 0;
    epd_get_display_size(&_w, &_h);

    obdCreateVirtualDisplay(&obd, _w, _h, epd_temp);
    obdFill(&obd, 0, 0); // fill with white

    char buff[100];
    battery_level = get_battery_level(battery_mv);

    // MAC
    sprintf(buff, "ESL_%02X%02X%02X", mac_public[2], mac_public[1], mac_public[0]);
    obdWriteStringCustom(&obd, (GFXfont *)&Dialog_plain_16, 1, 17, (char *)buff, 1);

    // 蓝牙连接状态
    if (ble_get_connected()) {
        sprintf(buff, "78%s", "234");
    } else {
        sprintf(buff, "78%s", "56");
    }
    obdWriteStringCustom(&obd, (GFXfont *)&Dialog_plain_16_zh, 110, 21, (char *)buff, 1);


    //电池电量
    obdRectangle(&obd, 218, 8, 220, 14, 1, 1); // 正极帽
    obdRectangle(&obd, 220, 4, 248, 18, 1, 1); // 电池体
    sprintf(buff, "%d", battery_level);
    obdWriteStringCustom(&obd, (GFXfont *)&Orbitron_Medium_9, 225, 15, (char *)buff, 0);

    // 横线
    obdRectangle(&obd, 0, 25, 248, 27, 1, 1);
    // 时间
    sprintf(buff, "%02d:%02d", dt->tm_hour, dt->tm_min);
    obdWriteStringCustom(&obd, (GFXfont *)&DSEG14_Classic_Mini_Regular_40, 15, 80, (char *)buff, 1);

    // 温度
    sprintf(buff, "   %d'C", EPD_read_temp());
    obdWriteStringCustom(&obd, (GFXfont *)&Dialog_plain_16, 175, 50, (char *)buff, 1);

    // 短横线
    obdRectangle(&obd, 170, 60, 248, 62, 1, 1);

    // 电压值
    sprintf(buff, "%dmV", battery_mv);
    obdWriteStringCustom(&obd, (GFXfont *)&Dialog_plain_16, 175, 84, (char *)buff, 1);

    // 竖线
    obdRectangle(&obd, 170, 27, 172, 99, 1, 1);
    // 下面的横线
    obdRectangle(&obd, 0, 97, 248, 99, 1, 1);

    // 日期
    sprintf(buff, "%d-%02d-%02d", dt->tm_year, dt->tm_month, dt->tm_day);
    obdWriteStringCustom(&obd, (GFXfont *)&Dialog_plain_16, 1, 118, (char *)buff, 1);
    // 星期
    uint8_t week = get_week(dt);
    if (week == 7) {
        sprintf(buff, "9:%c", week + 0x20 + 6);
    } else {
        sprintf(buff, "9:%c", week + 0x20);
    }
    obdWriteStringCustom(&obd, (GFXfont *)&Dialog_plain_16_zh, 100, 118, (char *)buff, 1);

    // 心灵鸡汤
    if (dt->tm_hour > 7 && dt->tm_hour < 20) {
        sprintf(buff, "%s", "EFGH");
    } else {
        sprintf(buff, "%s", "ABCD");
    }
    obdWriteStringCustom(&obd, (GFXfont *)&Dialog_plain_16_zh, 155, 118, (char *)buff, 1);

    FixBuffer(epd_temp, epd_buffer, _w, _h);

    EPD_Display(epd_buffer, NULL, _w * _h / 8, full_or_partial);
}