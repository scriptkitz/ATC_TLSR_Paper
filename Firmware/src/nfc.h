#pragma once
#include <stdint.h>
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


void init_nfc(void);
void nfc_loop(void);