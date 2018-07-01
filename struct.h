#ifndef STRUCT_H_
#define STRUCT_H_

#include "type.h"

struct main {
        int type;
        int data;
};

struct frame_can {
        u8 src;
        u8 dest;
        u8 form;
        u8 prio;
        u8 data[8];
        u32 tsc;
        struct frame_can *next;
};

struct frame_lvl_rx {
        u8 src;
        u8 dest;
        u8 form;
        u8 prio;
        union {
                u8 fault[8];
                struct {
                        s16 x;
                        s16 y;
                        u8 res0;
                        u8 res1;
                        u8 fault;
                        u8 xor;
                } state;
        } data;
        u32 tsc;
        struct frame_lvl_rx *next;
};

struct frame_lvl_tx {
        u8 src;
        u8 dest;
        u8 form;
        u8 prio;
        union {
                u8 fault[8];
                u8 query[8];
                struct {
                        int update;
                        int not;
                } cmd;
        } data;
        u32 tsc;
        struct frame_lvl_tx *next;
};

struct frame_vsl_rx {
        u8 src;
        u8 dest;
        u8 form;
        u8 prio;
        union {
                u8 fault[8];
                struct {
                        s16 x;
                        s16 y;
                        s16 z;
                        u8 fault;
                        u8 proc;
                } state;
        } data;
        u32 tsc;
        struct frame_vsl_rx *next;
};

struct frame_vsl_tx {
        u8 src;
        u8 dest;
        u8 form;
        u8 prio;
        union {
                u8 fault[8];
                u8 query[8];
                struct {
                        s16 x;
                        s16 y;
                        s16 z;
                        u8 mode;
                        u8 proc;
                } cmd;
        } data;
        u32 tsc;
        struct frame_vsl_tx *next;
};

struct frame_gen_rx {
        u8 src;
        u8 dest;
        u8 form;
        u8 prio;
        union {
                struct {
                        u8 tick;
                        u16 pow;
                        u16 fault;
                        u8 res0;
                        u8 res1;
                        u8 res2;
                } misc;
                struct {
                        u16 volt;
                        u16 ampr;
                        u16 rpm;
                        u8 temp;
                        u8 res0;
                } state;
                u8 fault[8];
        } data;
        u32 tsc;
        struct frame_gen_rx *next;
};

struct frame_gen_tx {
        u8 src;
        u8 dest;
        u8 form;
        u8 prio;
        union {
                u8 fault[8];
                u8 query[8];
                struct {
                        u8 type;
                        u8 enable;
                        u8 toggle;
                        u8 protect;
                        u8 res0;
                        u8 res1;
                        u8 res2;
                        u8 res3;
                } cmd;
        } data;
        u32 tsc;
        struct frame_gen_tx *next;
};

struct frame_psu_rx {
        u8 src;
        u8 dest;
        u8 form;
        u8 prio;
        union {
                u8 fault[8];
                struct {
                        u8 brake;
                        u8 light;
                        u16 v24;
                        u16 v500;
                        u8 fault;
                        u8 xor;
                } io;
                struct {
                        u16 volt_24;
                        u16 ampr_24;
                        u16 volt_500;
                        u16 ampr_500;
                } state;
        } data;
        u32 tsc;
        struct frame_psu_rx *next;
};

struct frame_psu_tx {
        u8 src;
        u8 dest;
        u8 form;
        u8 prio;
        union {
                u8 fault[8];
                u8 query[8];
                struct {
                        u8 brake;
                        u8 light;
                        u16 v24;
                        u16 v500;
                        u8 res;
                        u8 xor;
                } io;
        } data;
        u32 tsc;
        struct frame_psu_tx *next;
};

struct frame_cyl_rx {
        u8 src;
        u8 dest;
        u8 form;
        u8 prio;
        union {
                u8 fault[8];
                struct {
                        s16 pos;
                        s16 vel;
                        s16 ampr;
                        u8 fault;
                        u8 io;
                } state;
        } data;
        u32 tsc;
        struct frame_cyl_rx *next;
};

struct frame_cyl_tx {
        u8 src;
        u8 dest;
        u8 form;
        u8 prio;
        union {
                u8 fault[8];
                u8 query[8];
                struct {
                        s16 pos;
                        s16 vel;
                        s16 ampr;
                        u8 exec;
                        u8 enable;
                } cmd;
        } data;
        u32 tsc;
        struct frame_cyl_tx *next;
};

struct frame_udp_rx {
        int head;
        struct main cmd;
        u8 res0;
        u8 res1;
        u8 res2;
        u8 xor;
};

struct frame_udp_tx {
        int head;
        int res0;
        struct {
                u8 src;
                u8 dest;
                u8 form;
                u8 prio;
                u8 data[8];
                u32 tsc;
        } dev[50];
        int res1;
        int res2;
};

struct plan {
        int low;
        int acc;
        int high;
};

#endif /* STRUCT_H_ */
