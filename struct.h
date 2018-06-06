#ifndef STRUCT_H_
#define STRUCT_H_

#include "type.h"

struct main {
        int type;
        int data;
};

struct udp_cmd {
        u16 head;
        u8 res0;
        u8 res1;
        struct main cmd;
        u8 res2;
        u8 res3;
        u8 res4;
        u8 check;
};

struct frame_can {
        u8 src;
        u8 dest;
        u8 form;
        u8 prio;
        u8 data[8];
        u32 tsc;
};

#endif /* STRUCT_H_ */
