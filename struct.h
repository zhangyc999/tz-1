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

struct frame_psu_rx {
        u8 src;
        u8 dest;
        u8 form;
        u8 prio;
        union {
                u8 fault[8];
                struct {
                        u32 v24;
                        u16 v500;
                        u8 fault;
                        u8 xor;
                } io;
                struct {
                        u16 volt24;
                        u16 ampr24;
                        u16 volt500;
                        u16 ampr500;
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
                        u32 v24;
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

#endif /* STRUCT_H_ */
