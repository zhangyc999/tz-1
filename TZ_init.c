#include "struct.h"
#include "type.h"
#include "vx.h"

#define DUMMY

#define SYS_TICK_PER_SEC 200

extern void t_main(void);
extern void t_can(void);
extern void t_psu(void);
extern void t_mom(void);
extern void t_swh(void);
extern void t_rse(void);
extern void t_swv(void);
extern void t_prp(void);
extern void t_x(void);
extern void t_dbg(void);
extern void udp_server(void);

#ifdef DUMMY
extern void t_dum(void);
#endif

MSG_Q_ID msg_main;
MSG_Q_ID msg_gen;
MSG_Q_ID msg_psu;
MSG_Q_ID msg_mom;
MSG_Q_ID msg_swh;
MSG_Q_ID msg_rse;
MSG_Q_ID msg_swv;
MSG_Q_ID msg_prp;
MSG_Q_ID msg_x;
RING_ID rng_can[2];
RING_ID rng_udp;
RING_ID rng_dbg;

void tz(void)
{
        sysClkRateSet(SYS_TICK_PER_SEC);
        msg_main = msgQCreate(32, sizeof(struct main), MSG_Q_FIFO);
        msg_gen = msgQCreate(4, sizeof(struct frame_can), MSG_Q_FIFO);
        msg_psu = msgQCreate(4, sizeof(struct frame_can), MSG_Q_FIFO);
        msg_mom = msgQCreate(4, sizeof(struct frame_can), MSG_Q_FIFO);
        msg_swh = msgQCreate(4, sizeof(struct frame_can), MSG_Q_FIFO);
        msg_rse = msgQCreate(4, sizeof(struct frame_can), MSG_Q_FIFO);
        msg_swv = msgQCreate(4, sizeof(struct frame_can), MSG_Q_FIFO);
        msg_prp = msgQCreate(4, sizeof(struct frame_can), MSG_Q_FIFO);
        msg_x = msgQCreate(4, sizeof(struct frame_can), MSG_Q_FIFO);
        rng_can[0] = rngCreate(64 * sizeof(struct frame_can));
        rng_can[1] = rngCreate(64 * sizeof(struct frame_can));
        rng_udp = rngCreate(128 * sizeof(struct frame_can));
        rng_dbg = rngCreate(128 * sizeof(struct frame_can));
        taskSpawn("MAIN", 90, VX_FP_TASK, 20000, (FUNCPTR)t_main, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
#ifndef DUMMY
        taskSpawn("CAN", 90, VX_FP_TASK, 20000, (FUNCPTR)t_can, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
#endif /* DUMMY */
        taskSpawn("PSU", 90, VX_FP_TASK, 20000, (FUNCPTR)t_psu, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        taskSpawn("MOM", 90, VX_FP_TASK, 20000, (FUNCPTR)t_mom, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        taskSpawn("SWH", 90, VX_FP_TASK, 20000, (FUNCPTR)t_swh, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        taskSpawn("RSE", 90, VX_FP_TASK, 20000, (FUNCPTR)t_rse, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        taskSpawn("SWV", 90, VX_FP_TASK, 20000, (FUNCPTR)t_swv, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        taskSpawn("PRP", 90, VX_FP_TASK, 20000, (FUNCPTR)t_prp, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        taskSpawn("X", 90, VX_FP_TASK, 20000, (FUNCPTR)t_x, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
#ifdef DUMMY
        taskSpawn("DUM", 90, VX_FP_TASK, 20000, (FUNCPTR)t_dum, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
#endif /* DUMMY */
        taskSpawn("DBG", 100, VX_FP_TASK, 20000, (FUNCPTR)t_dbg, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        udp_server();
}
