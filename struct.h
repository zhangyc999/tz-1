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

#endif /* STRUCT_H_ */
