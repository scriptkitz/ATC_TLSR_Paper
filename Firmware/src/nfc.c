#include <stdint.h>
#include "common/utility.h"
#include "drivers/8258/timer.h"
#include "nfc.h"
#include "main.h"
#include "apdu.h"
#include "epd.h"

/**
 * ======================================================================================
 * 【核心知识点：FM11NC08I NFC 标签芯片】
 * ======================================================================================
 * 1. 身份切换机制 (重要！):
 *    该芯片在 I2C 模式下，CSN (对应代码中的 NFC_CS) 引脚决定了地址空间的映射：
 *    - 当 NFC_CS = 0 时: 访问 EEPROM 空间 (0x0000 - 0x03FF)。用于存储永久配置、ATQA、SAK 等。
 *    - 当 NFC_CS = 1 时: 访问 寄存器/FIFO 空间 (0xFFF0 - 0xFFFF)。用于实时 NFC 通讯。
 *    这就是为什么以前读不到数据：必须在访问寄存器和接收数据时将 NFC_CS 拉高。
 * 
 * 2. 模拟 T4T (Type 4 Tag):
 *    设备模拟成一个支持 ISO7816 APDU 指令的智能卡。手机的操作流程通常是：
 *    SELECT (选择 AID) -> SELECT (选择 CC 文件) -> READ (读 CC) -> SELECT (选择 NDEF) -> READ/WRITE。
 * ======================================================================================
 */

#define C081_NFC_ADDR       0xAE // I2C 8位写地址
#define FM11NC08I_FIFO			0xFFF0
#define FIFO_FLUSH_REG			0xFFF1
#define	FIFO_WORDCNT_REG		0xFFF2
#define RF_STATUS_REG			0xFFF3
#define RF_TXEN_REG				0xFFF4
#define MAIN_IRQ_REG			0xFFF7
#define FIFO_IRQ_REG			0xFFF8
#define AUX_IRQ_REG				0xFFF9
#define NFC_CFG_REG				0xFFFD

// MAIN_IRQ 位定义：用于判断手机的动作
#define MAIN_IRQ_RF_PWON        0x80 // 手机靠近（射频场上电）
#define MAIN_IRQ_ACTIVE         0x40 // 协议层已激活 (ISO 14443-4)
#define MAIN_IRQ_RX_START       0x20 // 手机开始发送数据包
#define MAIN_IRQ_RX_DONE        0x10 // 一帧数据接收完成
#define MAIN_IRQ_TX_DONE        0x08 // 响应数据发送完成
#define MAIN_IRQ_FIFO           0x02 // FIFO 状态变动
#define MAIN_IRQ_AUX            0x01 // 辅助中断

// FIFO_IRQ 位定义
#define FIFO_IRQ_WL             0x08 // Water Level：FIFO 满 24 字节，提醒主控快读
#define FIFO_IRQ_OVERFLOW       0x04 // FIFO溢出中断（包含上溢出和下溢出？读溢出写溢出？）
#define FIFO_IRQ_FULL           0x02 // FIFO满中断
#define FIFO_IRQ_EMPTY          0x01 // FIFO空中断

// AUX_IRQ
#define AUX_IRQ_EE_PROG_DONE    0x80 // EEPROM 编程完成中断
#define AUX_IRQ_EE_PROG_ERROR   0x40 // EEPROM 编程错误中断（权限错误）
#define AUX_IRQ_PARITY_ERROR    0x20 // 非接触端接收到的数据奇偶校验错误，仅在通过 FIFO 进行数据交互时有效
#define AUX_IRQ_CRC_ERROR       0x10 // RF 接收到的 CRC 校验错误，仅在通过 FIFO进行数据交互时有效
#define AUX_IRQ_FRAMING_ERROR   0x8  // RF 接收出现帧格式错误，仅在通过 FIFO 进行数据交互时有效


static uint8_t rx_frame_buff[256];
static uint32_t ndef_file_len = 0;
static uint32_t last_nfc_write_time = 0;
extern uint8_t epd_buffer[epd_buffer_size];

/**
 * 打印 NFC 内部寄存器状态（调试利器）
 */
void dump_nfc_status(uint8_t irq) {
    if (irq == 0) return;
    printf("NFC_STAT: 0x%02X [", irq);
    if (irq & MAIN_IRQ_RF_PWON) printf("RF_FIELD ");
    if (irq & MAIN_IRQ_ACTIVE)  printf("ISO_ACTIVE ");
    if (irq & MAIN_IRQ_RX_START) printf("RX_START ");
    if (irq & MAIN_IRQ_RX_DONE)  printf("RX_DONE ");
    printf("]\n");
}

/* 基础寄存器操作：确保 CSN=1 以选中寄存器区 */
void nfc_write_reg(uint16_t addr, uint8_t val) {
    gpio_write(NFC_CS, 1); 
    i2c_write_series(addr, 2, &val, 1);
    gpio_write(NFC_CS, 0); 
}

uint8_t nfc_read_reg(uint16_t addr) {
    gpio_write(NFC_CS, 1); 
    uint8_t r = i2c_read_byte(addr, 2);
    gpio_write(NFC_CS, 0);
    return r;
}

/* FIFO 数据操作 */
void nfc_read_fifo(uint8_t num, uint8_t* buff) {
    gpio_write(NFC_CS, 1);
    i2c_read_series(FM11NC08I_FIFO, 2, buff, num);
    gpio_write(NFC_CS, 0);
}

void nfc_write_fifo(uint8_t* buff, uint8_t size) {
    gpio_write(NFC_CS, 1);
    i2c_write_series(FM11NC08I_FIFO, 2, buff, size);
    gpio_write(NFC_CS, 0);
}

/**
 * 非阻塞式接收一帧 APDU 数据
 * 这里的逻辑是：检查中断标志，如果有数据就从 FIFO 搬运到 RAM
 */
uint32_t nfc_data_recv(uint8_t * rbuf) {
    uint8_t irq = nfc_read_reg(MAIN_IRQ_REG);
    if (!irq || irq == 0xFF) return 0;

    dump_nfc_status(irq); // 恢复你原有的中断监控功能

    uint8_t rlen = 0;
    // 轮询 FIFO 直到一帧完成。注意：虽然外层是非阻塞的，但在收到 RX_START 
    // 后内部需要同步读完这一包，因为一包数据通常很短（毫秒级）。
    while (irq & (MAIN_IRQ_RX_START | MAIN_IRQ_FIFO | MAIN_IRQ_RX_DONE)) {
        if (irq & MAIN_IRQ_FIFO) {
            uint8_t fifo_irq = nfc_read_reg(FIFO_IRQ_REG);
            if (fifo_irq & FIFO_IRQ_WL) { // FIFO 渐满，先读出 24 字节
                nfc_read_fifo(24, &rbuf[rlen]);
                rlen += 24;
            }
        }
        if (irq & MAIN_IRQ_RX_DONE) { // 数据包结束
            uint8_t temp = nfc_read_reg(FIFO_WORDCNT_REG);
            if (temp > 0) {
                nfc_read_fifo(temp, &rbuf[rlen]);
                rlen += temp;
            }
            break; 
        }
        irq = nfc_read_reg(MAIN_IRQ_REG);
    }

    if (rlen <= 2) return 0;
    return rlen - 2; // 减去末尾 2 字节硬件自动生成的 CRC
}

typedef enum { NONE, CC_FILE, NDEF_FILE } T4T_FILE;
static T4T_FILE current_file = NONE;

// CC 文件内容：描述本标签的能力
const uint8_t capability_container[15] = {
    0x00, 0x0F,        // CCLEN
    0x20,              // Mapping Version 2.0
    0x00, 0xF6,        // MLe (最大读长度)
    0x00, 0xF6,        // MLc (最大写长度)
    0x04, 0x06,        // NDEF File TLV
    0xE1, 0x04,        // NDEF File ID
    0xFF, 0xFF,        // NDEF Max Size (这里设为 64KB 以支持大图)
    0x00, 0x00         // 可读可写权限
};

/**
 * 【智能解析：识别 NDEF 类型】
 * 本函数负责区分手机发来的快递到底是什么内容
 */
void process_received_ndef() {
    if (ndef_file_len < 3) return;

    // T4T 格式会在 NDEF 数据前加 2 字节长度前缀
    uint16_t ndef_data_len = (epd_buffer[0] << 8) | epd_buffer[1];
    if (ndef_data_len == 0) return;

    uint8_t *ndef_ptr = &epd_buffer[2]; // 真正的 NDEF 数据开始处
    uint8_t header = ndef_ptr[0];
    uint8_t tnf = header & 0x07;       // 提取 Type Name Format

    // 情况 1: NDEF Text Record (文字)
    if (tnf == 0x01 && ndef_ptr[1] == 1 && ndef_ptr[3] == 'T') {
        // 解析 Text 记录：跳过状态字节和语言代码 (如 "en")
        uint8_t lang_len = ndef_ptr[6] & 0x3F; 
        char *text = (char *)&ndef_ptr[6 + 1 + lang_len];
        uint32_t text_len = (header & 0x10) ? ndef_ptr[2] : 0; // 简化处理短记录
        
        // 渲染文字到屏幕
        printf("NFC: Identified Text Record.\n");
        epd_display_nfc_text(text);
    } 
    // 情况 2: MIME 类型 (通常用于图像)
    else if (tnf == 0x02) {
        printf("NFC: Identified MIME Record (Image).\n");
        EPD_Display(epd_buffer, NULL, epd_buffer_size, 1);
    }
    // 情况 3: 兜底处理 (如果是 Raw 数据或未知 NDEF，且长度足够，按图片刷新)
    else if (ndef_file_len > (epd_buffer_size / 2)) {
        printf("NFC: Unknown NDEF or Raw Data, assuming Image.\n");
        EPD_Display(epd_buffer, NULL, epd_buffer_size, 1);
    }

    ndef_file_len = 0; // 处理完毕，重置计数器
}

/**
 * 处理一帧 APDU 指令
 */
void nfc_data_process(uint8_t size) {
    uint8_t status_ok[3] = { rx_frame_buff[0], 0x90, 0x00 };
    uint8_t status_err[3] = { rx_frame_buff[0], 0x6A, 0x82 };

    // APDU 解析：rx_frame_buff[2] 是 INS 指令字节
    switch (rx_frame_buff[2]) {
        
        case 0xA4: // SELECT 指令
            // 恢复原有的 AID 和 FID 分支识别
            if (rx_frame_buff[3] == 0x04) { // Select by AID (如 D2760000850101)
                printf("NFC: AID Selected.\n");
                nfc_write_fifo(status_ok, 3);
            } else if (rx_frame_buff[3] == 0x00) { // Select by File ID
                if (memcmp(&rx_frame_buff[6], "\xE1\x03", 2) == 0) {
                    current_file = CC_FILE;
                    nfc_write_fifo(status_ok, 3);
                } else if (memcmp(&rx_frame_buff[6], "\xE1\x04", 2) == 0) {
                    current_file = NDEF_FILE;
                    nfc_write_fifo(status_ok, 3);
                } else {
                    nfc_write_fifo(status_err, 3);
                }
            }
            break;

        case 0xB0: // READ BINARY 指令
            {
                uint16_t offset = (rx_frame_buff[3] << 8) | rx_frame_buff[4];
                uint8_t len = rx_frame_buff[5];
                if (current_file == CC_FILE) {
                    nfc_write_fifo(status_ok, 1);
                    nfc_write_fifo((uint8_t*)capability_container + offset, len);
                    nfc_write_fifo(&status_ok[1], 2);
                } else if (current_file == NDEF_FILE) {
                    nfc_write_fifo(status_ok, 1);
                    nfc_write_fifo(epd_buffer + offset, len);
                    nfc_write_fifo(&status_ok[1], 2);
                } else {
                    nfc_write_fifo(status_err, 3);
                }
            }
            break;

        case 0xD6: // WRITE BINARY 指令 (图片/字符串上传的关键)
            {
                uint16_t offset = (rx_frame_buff[3] << 8) | rx_frame_buff[4];
                uint8_t len = rx_frame_buff[5];
                if (current_file == NDEF_FILE && (offset + len <= epd_buffer_size)) {
                    // 将数据碎片存入大缓冲区
                    memcpy(epd_buffer + offset, &rx_frame_buff[6], len);
                    if (offset + len > ndef_file_len) ndef_file_len = offset + len;
                    
                    last_nfc_write_time = clock_time(); // 记录写入时间，用于异步刷新判断
                    nfc_write_fifo(status_ok, 3);
                } else {
                    nfc_write_fifo(status_err, 3);
                }
            }
            break;

        default: // 其他未显式处理的指令，返回 OK 保持通讯不中断
            nfc_write_fifo(status_ok, 3);
            break;
    }

    // 触发硬件发送，将 FIFO 中的数据回发给手机
    nfc_write_reg(RF_TXEN_REG, 0x55);
}

static bool _nfc_inited = false;
void init_nfc(void) {
    if (_nfc_inited) return;
    
    // I2C 速率配置：400KHz
    i2c_gpio_set(I2C_GPIO_GROUP_C0C1);
    i2c_master_init(C081_NFC_ADDR, (uint8_t)(CLOCK_SYS_CLOCK_HZ / (4 * 400000)));
    
    // 引脚模式初始化
    gpio_set_func(NFC_CS, AS_GPIO);
    gpio_set_output_en(NFC_CS, 1);
    gpio_set_func(NFC_IRQ, AS_GPIO);
    gpio_set_input_en(NFC_IRQ, 1);
    
    // 复位 FM11NC08I：先拉低，再拉高访问寄存器
    gpio_write(NFC_CS, 0); 
    sleep_us(200); 

    // 基础配置：开启协议中断，清空缓存
    nfc_write_reg(NFC_CFG_REG, 0x03); 
    nfc_write_reg(FIFO_FLUSH_REG, 0xFF); 
    
    _nfc_inited = true;
    printf("NFC: System Initialized (CSN High Mode).\n");
}

void nfc_loop() {
    init_nfc();
    
    // 1. 处理即时通讯帧
    uint32_t size = nfc_data_recv(rx_frame_buff);
    if (size > 0) {
        nfc_data_process(size);
    }

    // 2. 异步传输完成检测
    // 如果已经收到了数据，且超过 500ms 手机没有新的写入动作，说明传输结束
    if (ndef_file_len > 0 && (clock_time() - last_nfc_write_time > 500 * CLOCK_16M_SYS_TIMER_CLK_1MS)) {
        printf("NFC: Transfer Finished (%d bytes). Parsing...\n", ndef_file_len);
        process_received_ndef();
    }
}
