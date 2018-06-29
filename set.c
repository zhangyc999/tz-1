#include "j1939.h"
#include "struct.h"
#include "type.h"
#include "vx.h"

extern RING_ID rng_can[];
extern SEM_ID sem_can[];

static struct frame_can tx;

void set_pause(void)
{
        semTake(sem_can[0], WAIT_FOREVER);
        semTake(sem_can[1], WAIT_FOREVER);
}

void set_play(void)
{
        semGive(sem_can[0]);
        semGive(sem_can[1]);
}

void input_passwd(u8 addr)
{
        tx.src = J1939_ADDR_GW;
        tx.dest = addr;
        tx.form = J1939_FORM_PASSWD;
        tx.prio = J1939_PRIO_PASSWD;
        tx.data[0] = 'X';
        tx.data[1] = 'I';
        tx.data[2] = 'N';
        tx.data[3] = 'L';
        tx.data[4] = 'E';
        tx.data[5] = '1';
        tx.data[6] = '1';
        tx.data[7] = '9';
        rngBufPut(rng_can[0], (char *)&tx, sizeof(tx));
        rngBufPut(rng_can[1], (char *)&tx, sizeof(tx));
}

void save_config(u8 addr)
{
        tx.src = J1939_ADDR_GW;
        tx.dest = addr;
        tx.form = J1939_FORM_PASSWD;
        tx.prio = J1939_PRIO_PASSWD;
        tx.data[0] = 0x00;
        tx.data[1] = 0x11;
        tx.data[2] = 0x22;
        tx.data[3] = 0x33;
        tx.data[4] = 0x44;
        tx.data[5] = 0x55;
        tx.data[6] = 0x66;
        tx.data[7] = 0x77;
        rngBufPut(rng_can[0], (char *)&tx, sizeof(tx));
        rngBufPut(rng_can[1], (char *)&tx, sizeof(tx));
}

void set_zero(u8 addr)
{
        tx.src = J1939_ADDR_MAIN;
        tx.dest = addr;
        tx.form = J1939_FORM_SERVO_ZERO;
        tx.prio = J1939_PRIO_SERVO_ZERO;
        tx.data[0] = 0xCA;
        tx.data[1] = 0xCA;
        tx.data[2] = 0xCA;
        tx.data[3] = 0xCA;
        tx.data[4] = 0x35;
        tx.data[5] = 0x35;
        tx.data[6] = 0x35;
        tx.data[7] = 0x35;
        rngBufPut(rng_can[0], (char *)&tx, sizeof(tx));
        rngBufPut(rng_can[1], (char *)&tx, sizeof(tx));
}

void query_fault(u8 addr)
{
        tx.src = J1939_ADDR_MAIN;
        tx.dest = addr;
        tx.form = J1939_FORM_FAULT;
        tx.prio = J1939_PRIO_FAULT;
        tx.data[0] = 0x00;
        tx.data[1] = 0x11;
        tx.data[2] = 0x22;
        tx.data[3] = 0x33;
        tx.data[4] = 0x44;
        tx.data[5] = 0x55;
        tx.data[6] = 0x66;
        tx.data[7] = 0x77;
        rngBufPut(rng_can[0], (char *)&tx, sizeof(tx));
        rngBufPut(rng_can[1], (char *)&tx, sizeof(tx));
}
