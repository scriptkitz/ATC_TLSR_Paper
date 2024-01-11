#include <stdint.h>
#include "tl_common.h"
#include "drivers.h"
#include "stack/ble/ble.h"
#include "drivers/8258/flash.h"
#include "nfc.h"
#include "main.h"


uint8_t fm327_fifo[128]={0};
uint8_t rfLen;




// uint32_t write_and_read(uint16_t addr, uint8_t *buff, uint8_t send_len, uint8_t read_len)
// {
    
// }

void nfc_read_eep(uint8_t *buff, uint16_t addr, uint16_t len)
{
    i2c_read_series(addr, 2, buff, len);
}

/*读取FIFO*/
void nfc_read_fifo(uint8_t num, uint8_t* buff)
{
    nfc_read_eep(buff, FM327_FIFO, num);
}

uint8_t nfc_read_reg(uint16_t addr)
{
    uint8_t r = i2c_read_byte(addr, 2);
    return r;
}
void nfc_write_reg(uint16_t addr, uint8_t val)
{
    i2c_write_series(addr, 2, &val, 1);
}

uint32_t nfc_data_recv(uint8_t * rbuf)
{
    uint8_t irq = 0;
    uint8_t ret = 0;
    uint8_t irq_data_wl = 0;
    uint8_t irq_data_in = 0;
    uint8_t rlen = 0;
    uint8_t temp = 0;
    
    // while (1)
    {
        irq_data_wl = 0;
        irq = nfc_read_reg(MAIN_IRQ); //查询中断标志
        printf("irq: %d\n", irq);
        if (irq & MAIN_IRQ_FIFO) {
            ret = nfc_read_reg(FIFO_IRQ);
            printf("FIFO_IRQ: %d\n", ret);
            if(ret & FIFO_IRQ_WL) irq_data_wl = 1;
        }
        if(irq & MAIN_IRQ_AUX)
        {
            nfc_read_reg(AUX_IRQ);
            nfc_write_reg(FIFO_FLUSH, 0xFF);
        }
        if(irq & MAIN_IRQ_RX_START)
        {
            irq_data_in = 1;
        }
        if(irq_data_in && irq_data_wl)
        {
            irq_data_wl = 0;
            nfc_read_fifo(24, &rbuf[rlen]);
            rlen += 24;
        }
        if(irq && MAIN_IRQ_RX_DONE)
        {
            temp = (uint8_t)(nfc_read_reg(FIFO_WORDCNT)&0x3F);  // 接收完成后，计算fifo有多少字节
            nfc_read_fifo(temp, &rbuf[rlen]); // 读取最后的数据
            rlen += temp;
            irq_data_in = 0;
            // break;
        }
        WaitMs(1);
    }
	if (rlen <= 2) {
        return 0;
    }  
	rlen -= 2;//2字节crc校验
	return rlen;
}

void nfc_t4t()
{

    uint8_t crc_err = 0;
    uint8_t status_ok[3] = { 0x02, 0x90, 0x00 };
    uint8_t status_word[3] = { 0x02, 0x6A, 0x82 };
    uint8_t status_word2[3] = { 0x02, 0x6A, 0x00 };

    if (crc_err)
    {
        printf("nfc_t4t!!!\n");
        // nfc_write_fifo();
        // nfc_write_reg();
        crc_err = 0;
    }
    else
    {
        status_ok[0] = fm327_fifo[0];
        status_word[0] = fm327_fifo[0];
        status_word2[0] = fm327_fifo[0];
    }
    
}

_attribute_ram_code_ void init_nfc(void)
{
    i2c_gpio_set(I2C_GPIO_GROUP_C0C1);
	i2c_master_init(C081_NFC_ADDR, (uint8_t)(CLOCK_SYS_CLOCK_HZ / (4 * 400000))); // 400kHZ
    
    gpio_setup_up_down_resistor(NFC_CS, PM_PIN_PULLUP_10K);
    gpio_setup_up_down_resistor(NFC_IRQ, PM_PIN_PULLUP_10K);
    gpio_set_func(NFC_CS, AS_GPIO);
    gpio_set_func(NFC_IRQ, AS_GPIO);
    gpio_set_output_en(NFC_CS, 1);
    gpio_set_input_en(NFC_CS, 0);
    gpio_set_output_en(NFC_IRQ, 0);
    gpio_set_input_en(NFC_IRQ, 1);
    // 先拉低上电,需要延迟至少100us
    // gpio_write(NFC_CS, 0);
    // sleep_us(200);

    // printf("MAIN_IRQ_REG:%d\n", nfc_read_reg(MAIN_IRQ_REG));
    // printf("MAIN_IRQ_MASK_REG:%d\n", nfc_read_reg(MAIN_IRQ_MASK_REG));
    // printf("AUX_IRQ_REG:%d\n", nfc_read_reg(AUX_IRQ_REG));
    // printf("NFC_CFG:%d\n", nfc_read_reg(NFC_CFG_REG));
    
    // printf("EE_REGU_CFG:%d\n", nfc_read_reg(0x391));
    // printf("EE_I2C_ADDR:%d\n", nfc_read_reg(0x3B3));

    // 44 00 04 20
    // uint8_t ee_l3_cfg[4] = {0};
    // i2c_read_series(232*4, 2, ee_l3_cfg, 4);
    // printf("EE_L3_CFG:\n");
    // printf("    ATQA:%02X %02X\n", ee_l3_cfg[0], ee_l3_cfg[1]);
    // printf("    SAK1:%02X\n", ee_l3_cfg[2]);
    // printf("    SAK2:%02X\n", ee_l3_cfg[3]);
    
    // 05 72 01 57 F7 A0 02 00
    // uint8_t ee_nfc_cfg[8] = {0};
    // i2c_read_series(236*4, 2, ee_nfc_cfg, 8);
    // printf("EE_NTF_CFG:\n");
    // printf("    ATS(TL TO):%02X %02X\n", ee_nfc_cfg[0], ee_nfc_cfg[1]);
    // printf("    NFT_CFG:%02X\n", ee_nfc_cfg[2]);
    // printf("    I2C_ADDR:%02X\n", ee_nfc_cfg[3]);
    // printf("    ATS TA:%02X\n", ee_nfc_cfg[4]);
    // printf("    ATS TB:%02X\n", ee_nfc_cfg[5]);
    // printf("    ATS TC:%02X\n", ee_nfc_cfg[6]);
    
    
    //              no limit      2mA resistor   3.3V
    uint8_t vout = (0b11 << 4) | (0b10 << 2) | (0b11 << 0);
    nfc_write_reg(VOUT_CFG_REG, vout);
    //                 ISO14443-3   NO_IRQ_PWON|NO_IRQ_L4
    uint8_t nfc_cfg = (0b11 << 2) | 0b11;
    nfc_write_reg(NFC_CFG_REG, vout);

}

void nfc_loop()
{
    init_nfc();
    rfLen = nfc_data_recv(fm327_fifo);
    printf("nfc_loop:%d\n", rfLen);
    if(rfLen > 0)
    {
        printf("fm327_fifo[0]:%d\n", fm327_fifo[0]);
        nfc_t4t();
    }
}