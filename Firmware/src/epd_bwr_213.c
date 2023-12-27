#include <stdint.h>
#include "tl_common.h"
#include "main.h"
#include "epd.h"
#include "epd_spi.h"
#include "epd_bwr_213.h"
#include "drivers.h"
#include "stack/ble/ble.h"

// SSD1675 mixed with SSD1680 EPD Controller

#define BWR_213_Len 50
uint8_t LUT_bwr_213_part[] = {

  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  BWR_213_Len, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 
0x00, 0x00, 0x00, 

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

_attribute_ram_code_ uint8_t EPD_BWR_213_read_temp(void)
{
    uint8_t epd_temperature = 0 ;
    
    EPD_CheckStatus(100);
    // SW Reset
    EPD_WriteCmd(0x12);
    EPD_CheckStatus(100);

    // Temperature sensor control
    EPD_WriteCmd(0x18);
    EPD_WriteData(0x80);

    // Display update control
    EPD_WriteCmd(0x22);
    EPD_WriteData(0xA1); // 启用CLK，并只加载温度
    
    // Master Activation
    EPD_WriteCmd(0x20);
    EPD_CheckStatus(5000);

    // Temperature sensor read from register
    EPD_WriteCmd(0x1B);
    epd_temperature = EPD_SPI_read();
    EPD_SPI_read();
    WaitMs(5);

    // deep sleep
    EPD_WriteCmd(0x10);
    EPD_WriteData(0x01);

    return epd_temperature;
}

// 0x22 命令
const unsigned char ENABLE_CLOCK  = 1 << 7; 
const unsigned char ENABLE_ANALOG = 1 << 6; 
const unsigned char LOAD_TEMPERATURE = 1 << 5; 
const unsigned char LOAD_LUT_WITH_DISPLAY_MODE1 = 1 << 4; 
const unsigned char LOAD_LUT_WITH_DISPLAY_MODE2 = 1 << 3; 
const unsigned char DISPLAY = 1 << 2; 
const unsigned char DISABLE_ANALOG = 1 << 1; 
const unsigned char DISABLE_CLOCK = 1 << 0; 


_attribute_ram_code_ uint8_t EPD_BWR_213_Display(unsigned char *image, int size, uint8_t full_or_partial)
{    
    uint8_t epd_temperature = 0 ;
    
    EPD_CheckStatus(20000);
    // SW Reset
    EPD_WriteCmd(0x12);
    EPD_CheckStatus(20000);

    // 先获取温度
    {
        // Temperature sensor control
        EPD_WriteCmd(0x18);
        EPD_WriteData(0x80);

        // Display update control
        EPD_WriteCmd(0x22);
        EPD_WriteData(0xB1); // 加载温度值，加载LUT with Display Mode 1
        
        // Master Activation
        EPD_WriteCmd(0x20);
        EPD_CheckStatus(20000);

        // Temperature sensor read from register
        EPD_WriteCmd(0x1B);
        epd_temperature = EPD_SPI_read();
        EPD_WriteCmd(0x7F);
        // int epd_temperature2 = EPD_SPI_read(); // 高4位，看SSD1680的6.8.3
        // printf("Temperature: %d\n", epd_temperature2);
        WaitMs(5);
    }

    EPD_CheckStatus(20000);

    // Driver output control
    EPD_WriteCmd(0x01);
    EPD_WriteData(0x27);  // 这个要填最大支持的295+1个MUX, 不能填250
    EPD_WriteData(0x01);
#define EPD_01_SM
#define EPD_01_TB 
    EPD_WriteData(0x01); // 这个为0就会导致横屏的水平反转了

    // Data entry mode setting
    EPD_WriteCmd(0x11);
    EPD_WriteData(0x01);

    // Set RAM X- Address Start/End
    EPD_WriteCmd(0x44);
    EPD_WriteData(0x00);
    EPD_WriteData(0x0F);

    // Set RAM Y- Address Start/End
    EPD_WriteCmd(0x45);
    EPD_WriteData(0x27); // 这个要填最大支持的295+1个MUX, 不能填250
    EPD_WriteData(0x01);
    EPD_WriteData(0x00);
    EPD_WriteData(0x00);

    // Border waveform control
    EPD_WriteCmd(0x3C);
    EPD_WriteData(0x05);

    // Display update control
    EPD_WriteCmd(0x21);
    EPD_WriteData(0x00);
    EPD_WriteData(0x80);

    // Set RAM X address
    EPD_WriteCmd(0x4E);
    EPD_WriteData(0x00);

    // Set RAM Y address
    EPD_WriteCmd(0x4F);
    EPD_WriteData(0x27); // 这个要填最大支持的295+1个MUX, 不能填250
    EPD_WriteData(0x01);

    // 写入黑白图
    EPD_LoadImage(image, size, 0x24);
    // EPD_WriteCmd(0x24);
    // uint8_t pcnt = 0;
    // for(uint16_t col=0; col<250; col++)
    // {
    //     if( col < 250/2)
    //         pcnt = 0;
    //     else
    //         pcnt = 0xFF;
    //     for(uint16_t row=0; row<16; row++)
    //     {
    //         EPD_WriteData(pcnt);
    //     }
    // }
    EPD_WriteCmd(0x7F);

    // 填充红色图
    EPD_WriteCmd(0x26);// RED Color TODO make something out of it :)
    for (int c = 0; c < size; c++)
    {
        EPD_WriteData(0x00);
    }
    EPD_WriteCmd(0x7F);

    // 局部刷新
    {
        int i;
        if (!full_or_partial)
        {
            EPD_WriteCmd(0x32);
            for (i = 0; i < sizeof(LUT_bwr_213_part); i++)
            {
                EPD_WriteData(LUT_bwr_213_part[i]);
            }
            EPD_WriteCmd(0x7F);
        }
    }

    // Display update control
    EPD_WriteCmd(0x22);
    EPD_WriteData(0xC7);
    
    printf("%d-%d\n", 1000, gpio_read(EPD_BUSY));
    // Master Activation
    EPD_WriteCmd(0x20);
    printf("%d-%d\n", 101, gpio_read(EPD_BUSY));
    EPD_CheckStatus(40000);
    printf("%d-%d\n", 102, gpio_read(EPD_BUSY));


    return epd_temperature;
}

_attribute_ram_code_ void EPD_BWR_213_set_sleep(void)
{
    // deep sleep
    EPD_WriteCmd(0x10);
    EPD_WriteData(0x01);
    WaitMs(100);
}