#include "struct.h"
#include "type.h"
#include "vx.h"

#define DUMMY

#define SYS_TICK_PER_SEC 100

void t_main(void);
void t_can(void);
void t_lvl(void);
void t_psu(void);
void t_mom(void);
void t_swh(void);
void t_rse(void);
void t_swv(void);
void t_prp(void);
void t_top(void);
void t_shd(void);
void t_x(void);
void t_y(void);
void t_z(void);
void t_dbg(void);
void udp_server(void);

#ifdef DUMMY
void t_dum(void);
#endif

MSG_Q_ID msg_main;
MSG_Q_ID msg_lvl;
MSG_Q_ID msg_vsl;
MSG_Q_ID msg_gen;
MSG_Q_ID msg_psu;
MSG_Q_ID msg_mom;
MSG_Q_ID msg_swh;
MSG_Q_ID msg_rse;
MSG_Q_ID msg_swv;
MSG_Q_ID msg_prp;
MSG_Q_ID msg_top;
MSG_Q_ID msg_shd;
MSG_Q_ID msg_x;
MSG_Q_ID msg_y;
MSG_Q_ID msg_z;
RING_ID rng_can_slow[2];
RING_ID rng_can_fast[2];
RING_ID rng_udp[2];
RING_ID rng_result;
RING_ID rng_dbg[2];
SEM_ID sem_can[2];
SEM_ID sem_result;

void tz(void)
{
        sysClkRateSet(SYS_TICK_PER_SEC);
        msg_main = msgQCreate(32, sizeof(struct main), MSG_Q_FIFO);
        msg_lvl = msgQCreate(16, sizeof(struct frame_can), MSG_Q_FIFO);
        msg_vsl = msgQCreate(16, sizeof(struct frame_can), MSG_Q_FIFO);
        msg_gen = msgQCreate(16, sizeof(struct frame_can), MSG_Q_FIFO);
        msg_psu = msgQCreate(16, sizeof(struct frame_can), MSG_Q_FIFO);
        msg_mom = msgQCreate(16, sizeof(struct frame_can), MSG_Q_FIFO);
        msg_swh = msgQCreate(16, sizeof(struct frame_can), MSG_Q_FIFO);
        msg_rse = msgQCreate(16, sizeof(struct frame_can), MSG_Q_FIFO);
        msg_swv = msgQCreate(16, sizeof(struct frame_can), MSG_Q_FIFO);
        msg_prp = msgQCreate(16, sizeof(struct frame_can), MSG_Q_FIFO);
        msg_top = msgQCreate(16, sizeof(struct frame_can), MSG_Q_FIFO);
        msg_shd = msgQCreate(16, sizeof(struct frame_can), MSG_Q_FIFO);
        msg_x = msgQCreate(16, sizeof(struct frame_can), MSG_Q_FIFO);
        msg_y = msgQCreate(16, sizeof(struct frame_can), MSG_Q_FIFO);
        msg_z = msgQCreate(16, sizeof(struct frame_can), MSG_Q_FIFO);
        rng_can_slow[0] = rngCreate(64 * sizeof(struct frame_can));
        rng_can_slow[1] = rngCreate(64 * sizeof(struct frame_can));
        rng_can_fast[0] = rngCreate(64 * sizeof(struct frame_can));
        rng_can_fast[1] = rngCreate(64 * sizeof(struct frame_can));
        rng_udp[0] = rngCreate(64 * sizeof(struct frame_can));
        rng_udp[1] = rngCreate(64 * sizeof(struct frame_can));
        rng_result = rngCreate(1024 * sizeof(int));
        rng_dbg[0] = rngCreate(64 * sizeof(struct frame_can));
        rng_dbg[1] = rngCreate(64 * sizeof(struct frame_can));
        sem_can[0] = semMCreate(SEM_Q_FIFO);
        sem_can[1] = semMCreate(SEM_Q_FIFO);
        sem_result = semMCreate(SEM_Q_FIFO);
        taskSpawn("MAIN", 90, VX_FP_TASK, 20000, (FUNCPTR)t_main, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
#ifndef DUMMY
        taskSpawn("CAN", 90, VX_FP_TASK, 20000, (FUNCPTR)t_can, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
#endif /* DUMMY */
        taskSpawn("LVL", 90, VX_FP_TASK, 20000, (FUNCPTR)t_lvl, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        taskSpawn("PSU", 90, VX_FP_TASK, 20000, (FUNCPTR)t_psu, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        taskSpawn("MOM", 90, VX_FP_TASK, 20000, (FUNCPTR)t_mom, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        taskSpawn("SWH", 90, VX_FP_TASK, 20000, (FUNCPTR)t_swh, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        taskSpawn("RSE", 90, VX_FP_TASK, 20000, (FUNCPTR)t_rse, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        taskSpawn("SWV", 90, VX_FP_TASK, 20000, (FUNCPTR)t_swv, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        taskSpawn("PRP", 90, VX_FP_TASK, 20000, (FUNCPTR)t_prp, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        taskSpawn("TOP", 90, VX_FP_TASK, 20000, (FUNCPTR)t_top, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        taskSpawn("SHD", 90, VX_FP_TASK, 20000, (FUNCPTR)t_shd, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        taskSpawn("X", 90, VX_FP_TASK, 20000, (FUNCPTR)t_x, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        taskSpawn("Y", 90, VX_FP_TASK, 20000, (FUNCPTR)t_y, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        taskSpawn("Z", 90, VX_FP_TASK, 20000, (FUNCPTR)t_z, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
#ifdef DUMMY
        taskSpawn("DUM", 90, VX_FP_TASK, 20000, (FUNCPTR)t_dum, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
#endif /* DUMMY */
        taskSpawn("DBG", 100, VX_FP_TASK, 20000, (FUNCPTR)t_dbg, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        udp_server();
}
