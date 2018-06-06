#include "addr.h"
#include "type.h"
#include "vx.h"

#define SYS_TICK_PER_SEC 200

struct frame_can {
        u8 src;
        u8 dest;
        u8 form;
        u8 prio;
        u8 data[8];
        u32 tsc;
};

struct main {
        int type;
        int data;
};

extern void t_main(void);
extern void t_udp(void);
extern void t_udp(void);
extern void t_can(void);
extern void t_swh(void);

int udp_socket;
MSG_Q_ID msg_can[2];
MSG_Q_ID msg_main;
MSG_Q_ID msg_pwr;
MSG_Q_ID msg_swh;

void tz(void)
{
        sysClkRateSet(SYS_TICK_PER_SEC);
        udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
        msg_can[0] = msgQCreate(64, sizeof(struct frame_can), MSG_Q_FIFO);
        msg_can[1] = msgQCreate(64, sizeof(struct frame_can), MSG_Q_FIFO);
        msg_main = msgQCreate(32, sizeof(struct main), MSG_Q_FIFO);
        msg_pwr = msgQCreate(4, sizeof(struct frame_can), MSG_Q_FIFO);
        msg_swh = msgQCreate(4, sizeof(struct frame_can), MSG_Q_FIFO);
        taskSpawn("MAIN", 90, VX_FP_TASK, 20000, (FUNCPTR)t_main, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        taskSpawn("UDP", 90, VX_FP_TASK, 20000, (FUNCPTR)t_udp, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        taskSpawn("CAN", 90, VX_FP_TASK, 20000, (FUNCPTR)t_can, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        taskSpawn("SWH", 90, VX_FP_TASK, 20000, (FUNCPTR)t_swh, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
}
