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

#endif /* STRUCT_H_ */
