#include <stdint.h>
#include "tl_common.h"
#include "drivers.h"
#include "stack/ble/ble.h"
#include "drivers/8258/flash.h"
#include "nfc.h"
#include "main.h"

#define C081_NFC_ADDR       0xAE // 7 bit slave device address  1010 111 0/1
#define I2C_WR              0x00
#define I2C_RD              0x01
#define C081_NFC_READ_ADDR  0xAF
#define C081NFC_WRITE_ADDR    (C081_NFC_ADDR|I2C_WR)
#define C081NFC_READ_ADDR     (C081_NFC_ADDR|I2C_RD)
#define FM11_E2_USER_ADDR   0x0010
#define FM11_E2_MANUF_ADDR  0x03FF
#define FM11_E2_BLOCK_SIZE  16


/*FM11NC08i 寄存器*/
#define FM327_FIFO				0xFFF0
#define FIFO_FLUSH_REG			0xFFF1
#define	FIFO_WORDCNT_REG		0xFFF2
#define RF_STATUS_REG			0xFFF3
#define RF_TXEN_REG				0xFFF4
#define RF_BAUD_REG				0xFFF5
#define RF_RATS_REG				0xFFF6
#define MAIN_IRQ_REG			0xFFF7
#define FIFO_IRQ_REG			0xFFF8
#define AUX_IRQ_REG				0xFFF9
#define MAIN_IRQ_MASK_REG		0xFFFA
#define FIFO_IRQ_MASK_REG		0xFFFB
#define AUX_IRQ_MASK_REG		0xFFFC
#define NFC_CFG_REG				0xFFFD
#define VOUT_CFG_REG			0xFFFE
#define EE_WR_CTRL_REG			0xFFFF


#define MAIN_IRQ				0xFFF7
#define FIFO_IRQ				0xFFF8
#define AUX_IRQ		    	    0xFFF9
#define MAIN_IRQ_MASK		    0xFFFA
#define FIFO_IRQ_MASK		    0xFFFB
#define AUX_IRQ_MASK	        0xFFFC
#define FIFO_FLUSH			    0xFFF1
#define	FIFO_WORDCNT		    0xFFF2



#define MAIN_IRQ_RF_PWON        0x80 
#define MAIN_IRQ_ACTIVE         0x40
#define MAIN_IRQ_RX_START       0x20
#define MAIN_IRQ_RX_DONE        0x10
#define MAIN_IRQ_TX_DONE        0x08
#define MAIN_IRQ_ARBIT          0x04
#define MAIN_IRQ_FIFO           0x02
#define MAIN_IRQ_AUX            0x01

#define FIFO_IRQ_WL             0x08

// APDU 
#define APDU_CLA                                 (1)
#define APDU_INS                                 (2)
#define APDU_P1                                  (3)    
#define APDU_P2                                  (4)
#define APDU_LC                                  (5)
#define APDU_DATA                                (6)

uint8_t fm327_fifo[128]={0};
uint8_t rfLen;



void dump_hex(const uint8_t* buff, int len)
{
    for (int i = 0; i < len; i++)
    {
        printf("%02X ", buff[i]);
    }
    printf("\n");
}

/*写FIFO*/
void nfc_write_fifo(uint8_t* buff, uint8_t size)
{
    // todo: size > 32
    i2c_write_series(FM327_FIFO, 2, buff, size);
}
/*读取FIFO*/
void nfc_read_fifo(uint8_t num, uint8_t* buff)
{
    i2c_read_series(FM327_FIFO, 2, buff, num);
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
    
    while (1)
    {
        irq_data_wl = 0;
        irq = nfc_read_reg(MAIN_IRQ); //查询中断标志
        if (!irq) break;
        printf("irq1: %d\n", irq);
        if (irq & MAIN_IRQ_FIFO) {
            ret = nfc_read_reg(FIFO_IRQ);
            if(ret) printf("FIFO_IRQ: %d\n", ret);
            if(ret & FIFO_IRQ_WL) irq_data_wl = 1;
        }
        if(irq & MAIN_IRQ_AUX)
        {
            printf("AUX_IRQ: %d\n", nfc_read_reg(AUX_IRQ));
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
        if(irq & MAIN_IRQ_RX_DONE)
        {
            printf("MAIN_IRQ_RX_DONE\n");
            temp = (uint8_t)(nfc_read_reg(FIFO_WORDCNT)&0x3F);  // 接收完成后，计算fifo有多少字节
            printf("MAIN_IRQ_RX_DONE: %d\n", temp);
            nfc_read_fifo(temp, &rbuf[rlen]); // 读取最后的数据
            printf("MAIN_IRQ_RX_DONE End\n");
            rlen += temp;
            irq_data_in = 0;
            break;
        }
        printf("irq2: %d\n", irq);
        WaitMs(1);
        printf("irq3: %d\n", irq);
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
    uint8_t nak_crc_err = 0x05;

    // SW1 SW2
    uint8_t status_ok[3] = { 0x02, 0x90, 0x00 }; //  0x90处理完成.  0x00: None
    uint8_t status_word[3] = { 0x02, 0x6A, 0x82 }; // 0x6A:处理出错. 0x82:File or application not found
    uint8_t status_word2[3] = { 0x02, 0x6A, 0x00 }; // 0x6A: 错误. 0x00: No info

    nfc_write_fifo(fm327_fifo, rfLen);
    if (crc_err)
    {
        printf("nfc_t4t!!!\n");
        nfc_write_fifo(&nak_crc_err, 1);
        nfc_write_reg(RF_TXEN_REG, 0x55);
        crc_err = 0;
    }
    else
    {
        status_ok[0] = fm327_fifo[0];
        status_word[0] = fm327_fifo[0];
        status_word2[0] = fm327_fifo[0];

        if (fm327_fifo[APDU_INS] == 0xA4) // SELECT 
        {
            if (fm327_fifo[APDU_P1] == 0x00) // Select MF, DF or EF 
            {

            }
            else if (fm327_fifo[APDU_P1] == 0x04) // Select by DF name
            {
                printf("Selected by DF name:");
                dump_hex(&fm327_fifo[APDU_DATA], fm327_fifo[APDU_LC]);
                nfc_write_fifo(status_ok, 3);
                nfc_write_reg(RF_TXEN_REG, 0x55);
            }
            else
            {
                // 直接返回OK
                nfc_write_fifo(status_ok, 3);
                nfc_write_reg(RF_TXEN_REG, 0x55);
            }
        }
    }
    
}

bool _bInit = false;
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

    //  ATQA  SAK1  SAK2
    // 44 00   04    20
    // uint8_t ee_l3_cfg[4] = {0};
    // i2c_read_series(232*4, 2, ee_l3_cfg, 4);
    // printf("EE_L3_CFG:\n");
    // printf("    ATQA:%02X %02X\n", ee_l3_cfg[0], ee_l3_cfg[1]);
    // printf("    SAK1:%02X\n", ee_l3_cfg[2]);
    // printf("    SAK2:%02X\n", ee_l3_cfg[3]);
    
    // TL T0       TA TB TC 
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
    
    // 部分需要写EEPROM才生效，部分直接改寄存器就生效？
    // //              no limit      2mA resistor   3.3V
    // uint8_t vout = (0b11 << 4) | (0b10 << 2) | (0b11 << 0);
    // nfc_write_reg(VOUT_CFG_REG, vout);

    // //       NO_IRQ_PWON|NO_IRQ_L4
    uint8_t nfc_cfg = 0b11;
    nfc_write_reg(NFC_CFG_REG, nfc_cfg);

}

void nfc_loop()
{
    init_nfc();
    rfLen = nfc_data_recv(fm327_fifo);
    printf("rfLen: %d\n", rfLen);
    // 14字节
    // 02 00 A4 04 00 07 D2 76 00 00 85 01 01 00
    if(rfLen) printf("nfc_loop2:%d\n", rfLen);
    if(rfLen > 0)
    {
        dump_hex(fm327_fifo, rfLen);
        nfc_t4t();
    }
}