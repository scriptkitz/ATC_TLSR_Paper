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

// MAIN_IRQ
#define MAIN_IRQ_RF_PWON        0x80 // 射频上电中断
#define MAIN_IRQ_ACTIVE         0x40 // ISO 14443-3 SELECT 完成中断
#define MAIN_IRQ_RX_START       0x20 // ISO 14443-3 SELECT 完成后开始接收数据中断
#define MAIN_IRQ_RX_DONE        0x10 // 当前数据帧接收完成中断
#define MAIN_IRQ_TX_DONE        0x08 // 当前数据帧回发完成中断
#define MAIN_IRQ_ARBIT          0x04 // 非接触口访问EEPROM时，如果接触口试图访问EEPROM则触发此中断
#define MAIN_IRQ_FIFO           0x02 // FIFO产生中断，需去查询FIFO中断标记存储器
#define MAIN_IRQ_AUX            0x01 // 辅助中断，如果为1去查询AUX_IRQ中断标记寄存器

// FIFO_IRQ
#define FIFO_IRQ_WL             0x08 // Water level中断（渐满中断）。FIFO接收数据超过一个阈值（这里是24字节）则触发中断
#define FIFO_IRQ_OVERFLOW       0x04 // FIFO溢出中断（包含上溢出和下溢出？读溢出写溢出？）
#define FIFO_IRQ_FULL           0x02 // FIFO满中断
#define FIFO_IRQ_EMPTY          0x01 // FIFO空中断

// AUX_IRQ
#define AUX_IRQ_EE_PROG_DONE    0x80 // EEPROM 编程完成中断
#define AUX_IRQ_EE_PROG_ERROR   0x40 // EEPROM 编程错误中断（权限错误）
#define AUX_IRQ_PARITY_ERROR    0x20 // 非接触端接收到的数据奇偶校验错误，仅在通过 FIFO 进行数据交互时有效
#define AUX_IRQ_CRC_ERROR       0x10 // RF 接收到的 CRC 校验错误，仅在通过 FIFO进行数据交互时有效
#define AUX_IRQ_FRAMING_ERROR   0x8  // RF 接收出现帧格式错误，仅在通过 FIFO 进行数据交互时有效

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
    gpio_write(NFC_CS, 1);
    // todo: size > 32
    i2c_write_series(FM327_FIFO, 2, buff, size);
    gpio_write(NFC_CS, 0);
}
/*读取FIFO*/
void nfc_read_fifo(uint8_t num, uint8_t* buff)
{
    gpio_write(NFC_CS, 1);
    i2c_read_series(FM327_FIFO, 2, buff, num);
    gpio_write(NFC_CS, 0);
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
    uint8_t fifo_irq = 0;
    uint8_t irq_data_wl = 0;
    uint8_t irq_data_in = 0;
    uint8_t rlen = 0;
    uint8_t temp = 0;
    
#define PIRQ(V,IRQ) if(V&IRQ)printf("   %s\n", #IRQ)

    while (1)
    {
        fifo_irq = 0;
        irq_data_wl = 0;
        irq = nfc_read_reg(MAIN_IRQ_REG); //查询中断标志
        if (!irq) break;
        if (irq == 0xFF){
            printf("irq 0xFF ERROR!\n");
            break;
        }
        printf("irq: %d\n", irq);
        if (irq & MAIN_IRQ_RF_PWON)
        {
            // 参考文档：4.2.5.2 读写FIFO操作
            // 进场后，需要FIFO操作，让CSN高电平
            // gpio_write(NFC_CS, 1);
        }
        {
            printf("MAIN_IRQ:\n");
            PIRQ(irq, MAIN_IRQ_RF_PWON  );
            PIRQ(irq, MAIN_IRQ_ACTIVE   );
            PIRQ(irq, MAIN_IRQ_RX_START );
            PIRQ(irq, MAIN_IRQ_RX_DONE  );
            PIRQ(irq, MAIN_IRQ_TX_DONE  );
            PIRQ(irq, MAIN_IRQ_ARBIT    );
            PIRQ(irq, MAIN_IRQ_FIFO     );
            PIRQ(irq, MAIN_IRQ_AUX      );
        }
        if (irq & MAIN_IRQ_FIFO) {
            fifo_irq = nfc_read_reg(FIFO_IRQ_REG);
            if(fifo_irq & FIFO_IRQ_WL) irq_data_wl = 1;

            printf("FIFO_IRQ:\n");
            {
                PIRQ(fifo_irq, FIFO_IRQ_WL       );
                PIRQ(fifo_irq, FIFO_IRQ_OVERFLOW );
                PIRQ(fifo_irq, FIFO_IRQ_FULL     );
                PIRQ(fifo_irq, FIFO_IRQ_EMPTY    );
            }
        }
        uint8_t aux_irq = irq & MAIN_IRQ_AUX ? nfc_read_reg(AUX_IRQ_REG) : 0;

        if(aux_irq)
        {
            printf("AUX_IRQ: %d\n", aux_irq);
            PIRQ(aux_irq, AUX_IRQ_EE_PROG_DONE );
            PIRQ(aux_irq, AUX_IRQ_EE_PROG_ERROR);
            PIRQ(aux_irq, AUX_IRQ_PARITY_ERROR );
            PIRQ(aux_irq, AUX_IRQ_CRC_ERROR    );
            PIRQ(aux_irq, AUX_IRQ_FRAMING_ERROR);
        }

        if(fifo_irq & FIFO_IRQ_OVERFLOW)
        {
            printf("!FIFO OVERFLOW! %d\n", nfc_read_reg(FIFO_WORDCNT_REG));
        }
        if(irq & MAIN_IRQ_RX_START)
        {
            irq_data_in = 1;
            // uint8_t rats = nfc_read_reg(RF_RATS_REG);
            // int fsd = 0;
            // #define FSDI(k,v) case k:{ fsd = v; break;}
            // switch((rats >> 4)&0x0F)
            // {
            //     FSDI(0, 16)
            //     FSDI(1, 24)
            //     FSDI(2, 32)
            //     FSDI(3, 40)
            //     FSDI(4, 48)
            //     FSDI(5, 64)
            //     FSDI(6, 96)
            //     FSDI(7, 128)
            //     default:
            //         fsd = 256;
            //         break;
            // }
            // printf("FSDI: %02X\n", rats>>4);
            // printf("CDI: %02X\n", rats&0x0F);
        }
        if(irq_data_in && irq_data_wl)
        {
            irq_data_wl = 0;
            printf("WATER LEVEL Read!\n");
            nfc_read_fifo(24, &rbuf[rlen]);
            rlen += 24;
        }
        if(irq & MAIN_IRQ_RX_DONE)
        {
            temp = nfc_read_reg(FIFO_WORDCNT_REG);  // 接收完成后，计算fifo有多少字节
            printf("MAIN_IRQ_RX_DONE: %d\n", temp);
            if (temp > 0 && temp < 32) 
            {
                nfc_read_fifo(temp, &rbuf[rlen]); // 读取最后的数据
                // nfc_write_reg(FIFO_FLUSH_REG, 0xFF);
                rlen += temp;
            }
            printf("MAIN_IRQ_RX_DONE End\n");
            irq_data_in = 0;
            break;
        }
    }
    if (rlen <= 2) {
        if (rlen > 0) printf("ERROR rlen: %d\n", rlen);
        return 0;
    }
    rlen -= 2;//2字节crc校验
    return rlen;
}

typedef enum {
    NONE, 
    CC_FILE,
    NDEF_FILE 
} T4T_FILE;

uint8_t capability_container[15] =
{   0x00, 0x0F,        //CCLEN  
    0x20,              //Mapping Version 
    0x00, 0xF6,        //MLe 必须是F6  写成FF超过256字节就会分帧  但是写成F6就不会分帧
    0x00, 0xF6,        //MLc 必须是F6  写成FF超过256字节就会分帧  但是写成F6就不会分帧
    0x04,              //NDEF消息格式 05的话就是私有
    0x06,              //NDEF消息长度
    0xE1, 0x04,        //NDEF FILE ID       NDEF的文件标识符
    0x03, 0x84,        //NDEF最大长度
    0x00,              //Read Access           可读
    0x00               //Write Access          可写
};

uint8_t ndef_file[1024] = { 
#ifdef HUAWEI_COM    
    /*http://wwww.huawei.com*/
    0x00,0x0F,                    
    0xD1,0x01,0x0B,0x55,
    0x01,0x68,0x75,0x61,
    0x77,0x65,0x69,0x2E,
    0x63,0x6F,0x6D,
#endif    
/*wechat*/
#ifdef  NFC_TAG_WECHAT
0x00,0x20,
0xd4, 0x0f,0x0e, 0x61, 0x6e, 0x64, 0x72, 0x6f,
0x69, 0x64,0x2e, 0x63, 0x6f, 0x6d, 0x3a, 0x70,
0x6b, 0x67,0x63, 0x6f, 0x6d, 0x2e, 0x74, 0x65,
0x6e, 0x63,0x65, 0x6e, 0x74, 0x2e, 0x6d, 0x6d,
#endif
#ifdef NFC_TAG_HISTREAMING
0x00,0x3d,
0xd4, 0x0f, 0x2b, 0x61, 0x6e, 0x64, 0x72, 0x6f,
0x69, 0x64, 0x2e, 0x63, 0x6f, 0x6d, 0x3a, 0x70,
0x6b, 0x67, 0x61, 0x70, 0x70, 0x6b, 0x69, 0x74,
0x2e, 0x6f, 0x70, 0x65, 0x6e, 0x73, 0x6f, 0x75,
0x72, 0x63, 0x65, 0x2e, 0x67, 0x69, 0x7a, 0x77,
0x69, 0x74, 0x73, 0x2e, 0x63, 0x6f, 0x6d, 0x2e, 
0x6d, 0x79, 0x61, 0x70, 0x70, 0x6c, 0x69, 0x63, 
0x61, 0x74, 0x69, 0x6f, 0x6e,
#endif
#ifdef NFC_TAG_TODAY_HEADLINE
0x00,0x2d,
0xd4, 0x0f, 0x1b, 0x61, 0x6e, 0x64, 0x72, 0x6f, 
0x69, 0x64, 0x2e, 0x63, 0x6f, 0x6d, 0x3a, 0x70, 
0x6b, 0x67, 0x63, 0x6f, 0x6d, 0x2e, 0x73, 0x73,
0x2e, 0x61, 0x6e, 0x64, 0x72, 0x6f, 0x69, 0x64,
0x2e, 0x61, 0x72, 0x74, 0x69, 0x63, 0x6c, 0x65, 
0x2e, 0x6e, 0x65, 0x77, 0x73, 
#endif

#ifdef NFC_TAG_TAOBAO
0x00,0x23, 
0xd4, 0x0f, 0x11, 0x61, 0x6e, 0x64, 0x72, 0x6f, 
0x69, 0x64, 0x2e, 0x63, 0x6f, 0x6d, 0x3a, 0x70, 
0x6b, 0x67, 0x63, 0x6f, 0x6d, 0x2e, 0x74, 0x61, 
0x6f, 0x62, 0x61, 0x6f, 0x2e, 0x74, 0x61, 0x6f, 
0x62, 0x61, 0x6f, 
#endif
#ifdef NFC_TAG_HUAWEI_SMART_LIFE
0x00,0x26, 
0xd4, 0x0f, 0x14, 0x61, 0x6e, 0x64, 0x72, 0x6f, 
0x69, 0x64, 0x2e, 0x63, 0x6f, 0x6d, 0x3a, 0x70, 
0x6b, 0x67, 0x63, 0x6f, 0x6d, 0x2e, 0x68, 0x75, 
0x61, 0x77, 0x65, 0x69, 0x2e, 0x73, 0x6d, 0x61, 
0x72, 0x74, 0x68, 0x6f, 0x6d, 0x65, 
#endif
};

T4T_FILE current_file = NONE;

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
            static uint8_t ndef_capability_container[] = {0xE1, 0x03};
            static uint8_t ndef_id[] = {0xE1, 0x04};
            if (fm327_fifo[APDU_P1] == 0x00) // Select MF, DF or EF 
            {
                if(fm327_fifo[APDU_LC] == sizeof(ndef_capability_container) 
                    && 0 == memcmp(ndef_capability_container, &fm327_fifo[APDU_DATA], fm327_fifo[APDU_LC])) //
                {
                    nfc_write_fifo(status_ok, 3);
                    nfc_write_reg(RF_TXEN_REG, 0x55);
                    current_file = CC_FILE;
                }
                else if(fm327_fifo[APDU_LC] == sizeof(ndef_id) 
                    && 0 == memcmp(ndef_id, &fm327_fifo[APDU_DATA], fm327_fifo[APDU_LC])) //
                {
                    nfc_write_fifo(status_ok, 3);
                    nfc_write_reg(RF_TXEN_REG, 0x55);
                    current_file = NDEF_FILE;
                }
                else
                {
                    printf("Can't suppert:");
                    dump_hex(&fm327_fifo[APDU_DATA], fm327_fifo[APDU_LC]);
                    nfc_write_fifo(status_word2, 3);
                    nfc_write_reg(RF_TXEN_REG, 0x55);
                    current_file = NONE;
                }
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
        else if(fm327_fifo[APDU_INS] == 0xB0) // READ BINARY
        {
            if (current_file == CC_FILE)
            {
                nfc_write_fifo(status_ok, 1);
                nfc_write_fifo(((uint8_t*)capability_container) + (fm327_fifo[APDU_P1] << 8) + fm327_fifo[APDU_P2], fm327_fifo[APDU_LC]);
                nfc_write_fifo(&status_ok[1], 2);
                nfc_write_reg(RF_TXEN_REG, 0x55);
            }
            else if(current_file == NDEF_FILE)
            {

            }
            else
            {

            }
        }
        else if(fm327_fifo[APDU_INS] == 0xB0) // WRITE_BINARY
        {

        }
        else
        {

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
    gpio_write(NFC_CS, 0);
    sleep_us(200);

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
    
    // ATS
    // TL T0 -  -  TA TB TC -
    // 05 72 01 57 F7 A0 02 00
    // TL 05 表示ATS有效字节5个字节长度
    // T0 72 (0b0111 0010) b8应该是0，如果为1表示这个字节是RFU. b7-b5:表示是否TC/TB/TA是否存在。
    //                  b4-b1是FSCI用于编码FSC。FSC定义了PICC能接收帧的最大程度。默认值2。
    //                  FSCI的值和FSC的字节对应关系：0:16, 1:24, 2:32, 3:40, 4:48, 5:64, 6:96, 7:128, 8:256, 9-F:RFU>256
    //                  这里的值是2，就表示最大支持32字节。
    // TA F7 (0b1111 0111) b8:仅支持2个方向相同的D（除数）。D值影响位持续时间。
    //                     b7-b5: PICC到PCD方向的位速率（DS）。
    //                     b4: 0. 如果为1表示RFU
    //                     b3-b1: PCD到PICC方向的位速率（DR）。
    // TB A0 (0b1010 0000) b8-b5: FWI, 编码了FWT。FWT定义了接收PCD数据后发送PICC数据之前的等待时间。
    //                     b4-b1: SFGI, 编码了一个乘数值用于定义SFGT。SFGT定义了发送ATS后准备接收下一帧的保护时间。
    // TC 02 (0b0000 0010) b8-b3: 0,否则为RFU
    //                     b2-b1:定义了在PICC支持的开端字段中的可选字段。允许PCD跳过已被指出被PICC支持的字段，但PICC不支持的字段应不被PCD传输。
    //                     b2: 是否支持CID。默认1
    //                     b1: 是否支持NAD。默认0
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
    if (0)
    {
        _bInit = true;
        i2c_write_byte(0x3B1, 2, 0x72); // 修改T0值，本来是0x72
    }

    nfc_write_reg(FIFO_FLUSH_REG, 0xFF);
}

void nfc_loop()
{
    init_nfc();
    while (1)
    {
        rfLen = nfc_data_recv(fm327_fifo);
        if(rfLen) printf("rfLen: %d\n", rfLen);
        // 14字节
        // 02 00 A4 04 00 07 D2 76 00 00 85 01 01 00
        if(rfLen > 0)
        {
            dump_hex(fm327_fifo, rfLen);
            nfc_t4t();
        }
        sleep_ms(1);
    }
    gpio_write(NFC_CS, 0);
}