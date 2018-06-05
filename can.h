#ifndef CAN_H_
#define CAN_H_

#include "type.h"
#include "vx.h"

#ifndef _ASMLANGUAGE
IMPORT u8 sysInumTbl[];
#endif /* _ASMLANGUAGE */

#define ADDR_CAN0     0xD1000
#define ADDR_CAN1     0xD3000
#define ADDR_CAN2     0xD5000
#define ADDR_CAN3     0xD7000
#define ADDR_GPIO_IN  0xD9000
#define ADDR_GPIO_OUT 0xDA000

#define PELI_MODE  0x00
#define PELI_CMR   0x01
#define PELI_SR    0x02
#define PELI_IR    0x03
#define PELI_IER   0x04
#define PELI_BTR0  0x06
#define PELI_BTR1  0x07
#define PELI_OCR   0x08
#define PELI_ALC   0x0B
#define PELI_ECC   0x0C
#define PELI_EWLR  0x0D
#define PELI_RXERR 0x0E
#define PELI_TXERR 0x0F
#define PELI_TXB0  0x10
#define PELI_TXB1  0x11
#define PELI_TXB2  0x12
#define PELI_TXB3  0x13
#define PELI_TXB4  0x14
#define PELI_TXB5  0x15
#define PELI_TXB6  0x16
#define PELI_TXB7  0x17
#define PELI_TXB8  0x18
#define PELI_TXB9  0x19
#define PELI_TXB10 0x1A
#define PELI_TXB11 0x1B
#define PELI_TXB12 0x1C
#define PELI_RXB0  0x10
#define PELI_RXB1  0x11
#define PELI_RXB2  0x12
#define PELI_RXB3  0x13
#define PELI_RXB4  0x14
#define PELI_RXB5  0x15
#define PELI_RXB6  0x16
#define PELI_RXB7  0x17
#define PELI_RXB8  0x18
#define PELI_RXB9  0x19
#define PELI_RXB10 0x1A
#define PELI_RXB11 0x1B
#define PELI_RXB12 0x1C
#define PELI_ACR0  0x10
#define PELI_ACR1  0x11
#define PELI_ACR2  0x12
#define PELI_ACR3  0x13
#define PELI_AMR0  0x14
#define PELI_AMR1  0x15
#define PELI_AMR2  0x16
#define PELI_AMR3  0x17
#define PELI_RMC   0x1D
#define PELI_RBSA  0x1E
#define PELI_CDR   0x1F

#define GPIO_IN1       0x01
#define GPIO_IN2       0x02
#define GPIO_IN3       0x04
#define GPIO_IN4       0x08
#define GPIO_OUT1      0x01
#define GPIO_OUT2      0x02
#define GPIO_OUT3      0x04
#define GPIO_OUT4      0x08
#define GPIO_LED1_OUT  0x10
#define GPIO_LED2_OUT  0x20
#define GPIO_LED3_OUT  0x40

#define write_byte(addr, data) \
{                              \
    *(u8 *)addr = data;        \
}

#define write_reg_byte(addr, reg, data) \
{                                       \
    *(u8 *)addr = (u8)reg;              \
    *(u8 *)(addr + 0x1000) = (u8)data;  \
}

#define read_reg_byte(addr, reg) \
({                               \
    *(u8 *)addr = (u8)reg;       \
    *(u8 *)(addr + 0x1000);      \
})

#endif /* CAN_H_ */
