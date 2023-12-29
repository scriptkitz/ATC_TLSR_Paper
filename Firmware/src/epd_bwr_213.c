#include <stdint.h>
#include "common/types.h"
#include "tl_common.h"
#include "main.h"
#include "epd.h"
#include "epd_spi.h"
#include "epd_bwr_213.h"
#include "drivers.h"
#include "stack/ble/ble.h"

// SSD1675 mixed with SSD1680 EPD Controller

// 0x22 命令
const unsigned char ENABLE_CLOCK  = 1 << 7; 
const unsigned char ENABLE_ANALOG = 1 << 6; 
const unsigned char LOAD_TEMPERATURE = 1 << 5; 
const unsigned char LOAD_LUT_WITH_DISPLAY_MODE1 = 1 << 4; 
const unsigned char LOAD_LUT_WITH_DISPLAY_MODE2 = 1 << 3; 
const unsigned char DISPLAY = 1 << 2; 
const unsigned char DISABLE_ANALOG = 1 << 1; 
const unsigned char DISABLE_CLOCK = 1 << 0; 




#define BWR_213_Len 50
uint8_t LUT_bwr_213_part[] = {
    /*
    LUT 0: 第1行，10组数据, Group0 - Group9
    0x40 = b'01_00_00_00'; VS[0A-L0] _ VS[0B-L0] _ VS[0C-L0] _ VS[0D-L0]; Group0
    0x00 = b'00_00_00_00'; VS[1A-L0] _ VS[1B-L0] _ VS[1C-L0] _ VS[1D-L0]; Group1

    LUT 1: 第2行
    0x80 = b'10_00_00_00'; VS[0A-L1] _ VS[0B-L1] _ VS[0C-L1] _ VS[0D-L1]; Group0
    ...
    LUT 4: 第5行
    ...
    TP[0] ...
    */
    // 
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    // TP[0A], TP[0B], TP[0C], TP[0D], RP[0]
    BWR_213_Len, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00,
};

#define EPD_BWR_213_test_pattern 0xA5
_attribute_ram_code_ uint8_t EPD_BWR_213_detect(void)
{
    // SW Reset
    EPD_WriteCmd(0x12);
    WaitMs(10);

    EPD_WriteCmd(0x32);
    int i;
    for (i = 0; i < 153; i++)// This model has a 159 bytes LUT storage so we test for that
    {
        EPD_WriteData(EPD_BWR_213_test_pattern);
    }
    EPD_WriteCmd(0x33);
    for (i = 0; i < 153; i++)
    {
        if(EPD_SPI_read() != EPD_BWR_213_test_pattern)
            return 0;
    }
    return 1;
}

_attribute_ram_code_ uint8_t _EPD_read_temp(bool bLoadLUT)
{
    uint8_t epd_temperature = 0 ;
    // Temperature sensor control
    EPD_WriteCmd(0x18);
    EPD_WriteData(0x80);

    // Display update control
    EPD_WriteCmd(0x22);
    uint8_t flag = 0xA1; // 启用CLK，并只加载温度
    if (bLoadLUT)
        flag |= LOAD_LUT_WITH_DISPLAY_MODE1; // 加载LUT
    EPD_WriteData(flag); 
    
    // Master Activation
    EPD_WriteCmd(0x20);
    EPD_CheckStatus(5000);

    // Temperature sensor read from register
    EPD_WriteCmd(0x1B);
    epd_temperature = EPD_SPI_read();
    EPD_SPI_read();
    WaitMs(5);

    return epd_temperature;
}

_attribute_ram_code_ uint8_t EPD_BWR_213_read_temp(void)
{
    uint8_t epd_temperature = 0 ;
    
    EPD_CheckStatus(100);
    // SW Reset
    EPD_WriteCmd(0x12);
    EPD_CheckStatus(100);

    epd_temperature = _EPD_read_temp(false);

    EPD_BWR_213_set_sleep();
    return epd_temperature;
}

_attribute_ram_code_ void _EPD_SetWindows(uint16_t x_start, uint16_t x_end, uint16_t y_start, uint16_t y_end)
{
    // Set RAM X- Address Start/End
    EPD_WriteCmd(0x44);
    EPD_WriteData((x_start >> 3) & 0xFF);
    EPD_WriteData((x_end   >> 3) & 0xFF);

    // Set RAM Y- Address Start/End
    EPD_WriteCmd(0x45);
    EPD_WriteData( y_start       & 0xFF); // 这个要填最大支持的295+1个MUX, 不能填250
    EPD_WriteData((y_start >> 8) & 0xFF);
    EPD_WriteData( y_end &       0xFF);
    EPD_WriteData((y_end >> 8) & 0xFF);
};

_attribute_ram_code_ void _EPD_SetCursor(uint16_t x, uint16_t y)
{
    // Set RAM X address
    EPD_WriteCmd(0x4E);
    EPD_WriteData(x);

    // Set RAM Y address
    EPD_WriteCmd(0x4F);
    EPD_WriteData(y & 0xFF); // 这个要填最大支持的295+1个MUX, 不能填250
    EPD_WriteData((y >> 8) & 0xFF);
}

// 这个值再研究吧，看看跟屏幕分辨率有什么关系吗122*250
#define EPD_WIDTH 128
#define EPD_HEIGHT 296

_attribute_ram_code_ void _EPD_Display_Init()
{
    EPD_CheckStatus(1000);
    // SW Reset
    EPD_WriteCmd(0x12);
    EPD_CheckStatus(1000);

    // Driver output control
    EPD_WriteCmd(0x01);
    EPD_WriteData((EPD_HEIGHT-1) & 0xFF);  // 这个要填最大支持的295+1个MUX, 不能填250
    EPD_WriteData(((EPD_HEIGHT-1) >> 8) & 0xFF);
#define EPD_01_SM
#define EPD_01_TB
    EPD_WriteData(0x01); // 这个为0就会导致横屏的水平反转了

    // Data entry mode setting
    EPD_WriteCmd(0x11);
    EPD_WriteData(0x01);

    // Border waveform control
    EPD_WriteCmd(0x3C);
    EPD_WriteData(0x05);

    // Display update control
    EPD_WriteCmd(0x21);
    EPD_WriteData(0x00);
    EPD_WriteData(0x80);
}

_attribute_ram_code_ uint8_t EPD_BWR_213_Display(unsigned char *image, int size, bool full_refresh)
{    
    uint8_t epd_temperature = 0 ;
    
    _EPD_Display_Init();

    // 读取温度的同时加载LUT with Display Mode 1, 下面就不用再次加载LUT代码了。
    // 如果是全刷则需要加载OTP里的默认全刷LUT. 
    epd_temperature = _EPD_read_temp(full_refresh);
    EPD_CheckStatus(1000);

    // 设置显示区域
    _EPD_SetWindows(0, EPD_WIDTH-1, EPD_HEIGHT-1, 0);

    _EPD_SetCursor(0, EPD_HEIGHT-1);

    // 写入黑白图
    EPD_LoadImage(image, size, 0x24);
    EPD_WriteCmd(0x7F);

    // 填充红色图
    EPD_WriteCmd(0x26);// RED Color TODO make something out of it :)
    for (int c = 0; c < size; c++)
    {
        EPD_WriteData(0x00);
    }
    EPD_WriteCmd(0x7F);

    // 快速刷新
    if (!full_refresh)
    {
        EPD_WriteCmd(0x32);
        for (int i = 0; i < sizeof(LUT_bwr_213_part); i++)
        {
            EPD_WriteData(LUT_bwr_213_part[i]);
        }
        EPD_WriteCmd(0x7F);
    }

    // Display update control
    EPD_WriteCmd(0x22);
    EPD_WriteData(0xC7);
    
    // Master Activation
    EPD_WriteCmd(0x20);
    // Check BUSY 
    if(!EPD_CheckStatus(5000))
    {
        // BUSY Timeout!
        return 0xFF;
    }

    return epd_temperature;
}

_attribute_ram_code_ void EPD_BWR_213_set_sleep(void)
{
    // deep sleep
    EPD_WriteCmd(0x10);
    EPD_WriteData(0x01);
    WaitMs(10);
}