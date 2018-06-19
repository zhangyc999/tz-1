#include "struct.h"
#include "type.h"
#include "vx.h"

#define DEBUG

#define SYS_TICK_PER_SEC 200

extern void t_main(void);
extern void t_can(void);
extern void t_psu(void);
extern void t_swh(void);
extern void t_rse(void);
extern void t_swv(void);
extern void t_prp(void);
extern void udp_server(void);

#ifdef DEBUG
extern void t_dm0(void);
extern void t_dm1(void);
extern void t_dbg(void);
#endif

MSG_Q_ID msg_can[2];
MSG_Q_ID msg_main;
MSG_Q_ID msg_gen;
MSG_Q_ID msg_psu;
MSG_Q_ID msg_swh;
MSG_Q_ID msg_rse;
MSG_Q_ID msg_swv;
MSG_Q_ID msg_prp;
MSG_Q_ID msg_udp;
MSG_Q_ID msg_dbg;

void tz(void)
{
        sysClkRateSet(SYS_TICK_PER_SEC);
        msg_can[0] = msgQCreate(64, sizeof(struct frame_can), MSG_Q_FIFO);
        msg_can[1] = msgQCreate(64, sizeof(struct frame_can), MSG_Q_FIFO);
        msg_main = msgQCreate(32, sizeof(struct main), MSG_Q_FIFO);
        msg_gen = msgQCreate(4, sizeof(struct frame_can), MSG_Q_FIFO);
        msg_psu = msgQCreate(4, sizeof(struct frame_can), MSG_Q_FIFO);
        msg_swh = msgQCreate(4, sizeof(struct frame_can), MSG_Q_FIFO);
        msg_rse = msgQCreate(4, sizeof(struct frame_can), MSG_Q_FIFO);
        msg_swv = msgQCreate(4, sizeof(struct frame_can), MSG_Q_FIFO);
        msg_prp = msgQCreate(4, sizeof(struct frame_can), MSG_Q_FIFO);
        msg_udp = msgQCreate(128, sizeof(struct frame_can), MSG_Q_FIFO);
        msg_dbg = msgQCreate(128, sizeof(struct frame_can), MSG_Q_FIFO);
        taskSpawn("MAIN", 90, VX_FP_TASK, 20000, (FUNCPTR)t_main, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
#ifndef DEBUG
        taskSpawn("CAN", 90, VX_FP_TASK, 20000, (FUNCPTR)t_can, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
#endif /* DEBUG */
        taskSpawn("PSU", 90, VX_FP_TASK, 20000, (FUNCPTR)t_psu, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        taskSpawn("SWH", 90, VX_FP_TASK, 20000, (FUNCPTR)t_swh, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        taskSpawn("RSE", 90, VX_FP_TASK, 20000, (FUNCPTR)t_rse, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        taskSpawn("SWV", 90, VX_FP_TASK, 20000, (FUNCPTR)t_swv, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        taskSpawn("PRP", 90, VX_FP_TASK, 20000, (FUNCPTR)t_prp, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
#ifdef DEBUG
        taskSpawn("DM0", 90, VX_FP_TASK, 20000, (FUNCPTR)t_dm0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        taskSpawn("DM1", 90, VX_FP_TASK, 20000, (FUNCPTR)t_dm1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        taskSpawn("DBG", 100, VX_FP_TASK, 20000, (FUNCPTR)t_dbg, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
#endif /* DEBUG */
        udp_server();
}
