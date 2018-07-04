#include "j1939.h"
#include "struct.h"
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

#define GPIO_IN1      0x01
#define GPIO_IN2      0x02
#define GPIO_IN3      0x04
#define GPIO_IN4      0x08
#define GPIO_OUT1     0x01
#define GPIO_OUT2     0x02
#define GPIO_OUT3     0x04
#define GPIO_OUT4     0x08
#define GPIO_LED1_OUT 0x10
#define GPIO_LED2_OUT 0x20
#define GPIO_LED3_OUT 0x40

#define WRITE_BYTE(addr, data) \
{                              \
        *(u8 *)addr = data;    \
}

#define WRITE_REG_BYTE(addr, reg, data)    \
{                                          \
        *(u8 *)addr = (u8)reg;             \
        *(u8 *)(addr + 0x1000) = (u8)data; \
}

#define READ_REG_BYTE(addr, reg) \
({                               \
        *(u8 *)addr = (u8)reg;   \
        *(u8 *)(addr + 0x1000);  \
})

MSG_Q_ID remap_addr_msg(u8 addr);

extern RING_ID rng_can[];
extern RING_ID rng_udp[];
extern RING_ID rng_dbg[];

const static int addr_can[4] = {ADDR_CAN0, ADDR_CAN1, ADDR_CAN2, ADDR_CAN3};
const static int irq_can[4] = {5, 7, 11, 12};

static void init_can(int i);
static void isr_can_rx0(void);
static void isr_can_rx1(void);
static void (*isr_can[2])(void) = {isr_can_rx0, isr_can_rx1};
static struct frame_can buf;
static u8 id[4];
static int i;

void t_can(void)
{
        init_can(0);
        init_can(1);
        for (;;) {
                taskDelay(1);
                for (i = 0; i < 2; i++) {
                        if (READ_REG_BYTE(addr_can[i], PELI_SR) & 0x80) {
                                init_can(i);
                                continue;
                        }
                        if (sizeof(buf) != rngBufGet(rng_can[i], (char *)&buf, sizeof(buf)))
                                continue;
#if 1
                        if (i == 0)
                                printf("\033[25;1HCAN0:%8d", rngNBytes(rng_can[0]));
                        if (i == 1)
                                printf("\033[25;16HCAN1:%8d", rngNBytes(rng_can[1]));
#endif
                        buf.tsc = tickGet();
                        id[0] = buf.src;
                        id[1] = buf.dest;
                        id[2] = buf.form;
                        id[3] = buf.prio;
                        *(int *)id <<= 3;
                        WRITE_REG_BYTE(addr_can[i], PELI_TXB0, 0x88);
                        WRITE_REG_BYTE(addr_can[i], PELI_TXB1, id[3]);
                        WRITE_REG_BYTE(addr_can[i], PELI_TXB2, id[2]);
                        WRITE_REG_BYTE(addr_can[i], PELI_TXB3, id[1]);
                        WRITE_REG_BYTE(addr_can[i], PELI_TXB4, id[0]);
                        WRITE_REG_BYTE(addr_can[i], PELI_TXB5, buf.data[0]);
                        WRITE_REG_BYTE(addr_can[i], PELI_TXB6, buf.data[1]);
                        WRITE_REG_BYTE(addr_can[i], PELI_TXB7, buf.data[2]);
                        WRITE_REG_BYTE(addr_can[i], PELI_TXB8, buf.data[3]);
                        WRITE_REG_BYTE(addr_can[i], PELI_TXB9, buf.data[4]);
                        WRITE_REG_BYTE(addr_can[i], PELI_TXB10, buf.data[5]);
                        WRITE_REG_BYTE(addr_can[i], PELI_TXB11, buf.data[6]);
                        WRITE_REG_BYTE(addr_can[i], PELI_TXB12, buf.data[7]);
                        WRITE_REG_BYTE(addr_can[i], PELI_CMR, 0x01);
                }
        }
}

static void isr_can_rx0(void)
{
        struct frame_can can;
        u8 id[4];
        MSG_Q_ID msg;
        if (READ_REG_BYTE(ADDR_CAN0, PELI_IR) != 0x01) {
                WRITE_REG_BYTE(ADDR_CAN0, PELI_CMR, 0x04);
                return;
        }
        if ((READ_REG_BYTE(ADDR_CAN0, PELI_SR) & 0x03) != 0x01) {
                WRITE_REG_BYTE(ADDR_CAN0, PELI_CMR, 0x04);
                return;
        }
        if (READ_REG_BYTE(ADDR_CAN0, PELI_RXB0) != 0x88) {
                WRITE_REG_BYTE(ADDR_CAN0, PELI_CMR, 0x04);
                return;
        }
        id[3] = READ_REG_BYTE(ADDR_CAN0, PELI_RXB1);
        id[2] = READ_REG_BYTE(ADDR_CAN0, PELI_RXB2);
        id[1] = READ_REG_BYTE(ADDR_CAN0, PELI_RXB3);
        id[0] = READ_REG_BYTE(ADDR_CAN0, PELI_RXB4);
        *(int *)id >>= 3;
        can.src = id[0];
        can.dest = id[1];
        if (can.dest != J1939_ADDR_MAIN) {
                WRITE_REG_BYTE(ADDR_CAN0, PELI_CMR, 0x04);
                return;
        }
        can.form = id[2];
        can.prio = id[3];
        can.data[0] = READ_REG_BYTE(ADDR_CAN0, PELI_RXB5);
        can.data[1] = READ_REG_BYTE(ADDR_CAN0, PELI_RXB6);
        can.data[2] = READ_REG_BYTE(ADDR_CAN0, PELI_RXB7);
        can.data[3] = READ_REG_BYTE(ADDR_CAN0, PELI_RXB8);
        can.data[4] = READ_REG_BYTE(ADDR_CAN0, PELI_RXB9);
        can.data[5] = READ_REG_BYTE(ADDR_CAN0, PELI_RXB10);
        can.data[6] = READ_REG_BYTE(ADDR_CAN0, PELI_RXB11);
        can.data[7] = READ_REG_BYTE(ADDR_CAN0, PELI_RXB12);
        WRITE_REG_BYTE(ADDR_CAN0, PELI_CMR, 0x04);
        can.tsc = tickGet();
        msg = remap_addr_msg(can.src);
        if (!msg)
                return;
        msgQSend(msg, (char *)&can, sizeof(can), NO_WAIT, MSG_PRI_NORMAL);
        rngBufPut(rng_udp[0], (char *)&can, sizeof(can));
        rngBufPut(rng_dbg[0], (char *)&can, sizeof(can));
}

static void isr_can_rx1(void)
{
        struct frame_can can;
        u8 id[4];
        MSG_Q_ID msg;
        if (READ_REG_BYTE(ADDR_CAN1, PELI_IR) != 0x01) {
                WRITE_REG_BYTE(ADDR_CAN1, PELI_CMR, 0x04);
                return;
        }
        if ((READ_REG_BYTE(ADDR_CAN1, PELI_SR) & 0x03) != 0x01) {
                WRITE_REG_BYTE(ADDR_CAN1, PELI_CMR, 0x04);
                return;
        }
        if (READ_REG_BYTE(ADDR_CAN1, PELI_RXB0) != 0x88) {
                WRITE_REG_BYTE(ADDR_CAN1, PELI_CMR, 0x04);
                return;
        }
        id[3] = READ_REG_BYTE(ADDR_CAN1, PELI_RXB1);
        id[2] = READ_REG_BYTE(ADDR_CAN1, PELI_RXB2);
        id[1] = READ_REG_BYTE(ADDR_CAN1, PELI_RXB3);
        id[0] = READ_REG_BYTE(ADDR_CAN1, PELI_RXB4);
        *(int *)id >>= 3;
        can.src = id[0];
        can.dest = id[1];
        if (can.dest != J1939_ADDR_MAIN) {
                WRITE_REG_BYTE(ADDR_CAN1, PELI_CMR, 0x04);
                return;
        }
        can.form = id[2];
        can.prio = id[3];
        can.data[0] = READ_REG_BYTE(ADDR_CAN1, PELI_RXB5);
        can.data[1] = READ_REG_BYTE(ADDR_CAN1, PELI_RXB6);
        can.data[2] = READ_REG_BYTE(ADDR_CAN1, PELI_RXB7);
        can.data[3] = READ_REG_BYTE(ADDR_CAN1, PELI_RXB8);
        can.data[4] = READ_REG_BYTE(ADDR_CAN1, PELI_RXB9);
        can.data[5] = READ_REG_BYTE(ADDR_CAN1, PELI_RXB10);
        can.data[6] = READ_REG_BYTE(ADDR_CAN1, PELI_RXB11);
        can.data[7] = READ_REG_BYTE(ADDR_CAN1, PELI_RXB12);
        WRITE_REG_BYTE(ADDR_CAN1, PELI_CMR, 0x04);
        can.tsc = tickGet();
        msg = remap_addr_msg(can.src);
        if (!msg)
                return;
        msgQSend(msg, (char *)&can, sizeof(can), NO_WAIT, MSG_PRI_NORMAL);
        rngBufPut(rng_udp[1], (char *)&can, sizeof(can));
        rngBufPut(rng_dbg[1], (char *)&can, sizeof(can));
}

static void init_can(int i)
{
        /*
          +-----------------------+
          | BPS     | BTR0 | BTR1 |
          +---------+------+------+
          | 5Kbps   | 0xBF | 0xFF |
          | 10Kbps  | 0x31 | 0x1C |
          | 20Kbps  | 0x18 | 0x1C |
          | 40Kbps  | 0x87 | 0xFF |
          | 50Kbps  | 0x09 | 0x1C |
          | 80Kbps  | 0x83 | 0Xff |
          | 100Kbps | 0x04 | 0x1C |
          | 125Kbps | 0x03 | 0x1C |
          | 200Kbps | 0x81 | 0xFA |
          | 250Kbps | 0x01 | 0x1C |
          | 400Kbps | 0x80 | 0xFA |
          | 500Kbps | 0x00 | 0x1C |
          | 666Kbps | 0x80 | 0xB6 |
          | 800Kbps | 0x00 | 0x16 |
          | 1Mbps   | 0x00 | 0x14 |
          +---------+------+------+
        */
        sysIntDisablePIC(irq_can[i]);
        WRITE_REG_BYTE(addr_can[i], PELI_MODE, 0x09);
        WRITE_REG_BYTE(addr_can[i], PELI_CMR, 0x0C);
        WRITE_REG_BYTE(addr_can[i], PELI_CDR, 0x88);
        WRITE_REG_BYTE(addr_can[i], PELI_IER, 0x09);
        WRITE_REG_BYTE(addr_can[i], PELI_ACR0, 0xFF);
        WRITE_REG_BYTE(addr_can[i], PELI_ACR1, 0xFF);
        WRITE_REG_BYTE(addr_can[i], PELI_ACR2, 0xFF);
        WRITE_REG_BYTE(addr_can[i], PELI_ACR3, 0xFF);
        WRITE_REG_BYTE(addr_can[i], PELI_AMR0, 0xFF);
        WRITE_REG_BYTE(addr_can[i], PELI_AMR1, 0xFF);
        WRITE_REG_BYTE(addr_can[i], PELI_AMR2, 0xFF);
        WRITE_REG_BYTE(addr_can[i], PELI_AMR3, 0xFF);
        WRITE_REG_BYTE(addr_can[i], PELI_BTR0, 0x04);
        WRITE_REG_BYTE(addr_can[i], PELI_BTR1, 0x1C);
        WRITE_REG_BYTE(addr_can[i], PELI_EWLR, 0x60);
        WRITE_REG_BYTE(addr_can[i], PELI_OCR, 0x1A);
        WRITE_REG_BYTE(addr_can[i], PELI_MODE, 0x08);
        intConnect(INUM_TO_IVEC(sysInumTbl[irq_can[i]]), (VOIDFUNCPTR)isr_can[i], 0);
        sysIntEnablePIC(irq_can[i]);
}
