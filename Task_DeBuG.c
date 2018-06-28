#include "j1939.h"
#include "struct.h"
#include "type.h"
#include "vx.h"

extern RING_ID rng_dbg[];

static struct frame_can can;
static u16 total[48];
static int i;

void t_dbg(void)
{
        printf("\
\033[1;1HVSLF   |----|--|-- -- -- -- -- -- -- --| LVL(0) |----|--|-- -- -- -- -- -- -- --\
\033[2;1HVSLB   |----|--|-- -- -- -- -- -- -- --| LVL(1) |----|--|-- -- -- -- -- -- -- --\
\033[3;1HSWH(0) |----|--|-- -- -- -- -- -- -- --| FX     |----|--|-- -- -- -- -- -- -- --\
\033[4;1HSWH(1) |----|--|-- -- -- -- -- -- -- --| BX     |----|--|-- -- -- -- -- -- -- --\
\033[5;1HSWH(2) |----|--|-- -- -- -- -- -- -- --| FZ     |----|--|-- -- -- -- -- -- -- --\
\033[6;1HSWH(3) |----|--|-- -- -- -- -- -- -- --| BZ     |----|--|-- -- -- -- -- -- -- --\
\033[7;1HRSE(0) |----|--|-- -- -- -- -- -- -- --| MOM(0) |----|--|-- -- -- -- -- -- -- --\
\033[8;1HRSE(1) |----|--|-- -- -- -- -- -- -- --| MOM(1) |----|--|-- -- -- -- -- -- -- --\
\033[9;1HRSE(2) |----|--|-- -- -- -- -- -- -- --| MOM(2) |----|--|-- -- -- -- -- -- -- --\
\033[10;1HRSE(3) |----|--|-- -- -- -- -- -- -- --| MOM(3) |----|--|-- -- -- -- -- -- -- --\
\033[11;1HSWV(0) |----|--|-- -- -- -- -- -- -- --| TOP    |----|--|-- -- -- -- -- -- -- --\
\033[12;1HSWV(1) |----|--|-- -- -- -- -- -- -- --| SDS(0) |----|--|-- -- -- -- -- -- -- --\
\033[13;1HSWV(2) |----|--|-- -- -- -- -- -- -- --| SDS(1) |----|--|-- -- -- -- -- -- -- --\
\033[14;1HSWV(3) |----|--|-- -- -- -- -- -- -- --| SDS(2) |----|--|-- -- -- -- -- -- -- --\
\033[15;1HPRP(0) |----|--|-- -- -- -- -- -- -- --| SDS(3) |----|--|-- -- -- -- -- -- -- --\
\033[16;1HPRP(1) |----|--|-- -- -- -- -- -- -- --| SDF(0) |----|--|-- -- -- -- -- -- -- --\
\033[17;1HPRP(2) |----|--|-- -- -- -- -- -- -- --| SDF(1) |----|--|-- -- -- -- -- -- -- --\
\033[18;1HPRP(3) |----|--|-- -- -- -- -- -- -- --| SDF(2) |----|--|-- -- -- -- -- -- -- --\
\033[19;1HFY(0)  |----|--|-- -- -- -- -- -- -- --| SDF(3) |----|--|-- -- -- -- -- -- -- --\
\033[20;1HFY(1)  |----|--|-- -- -- -- -- -- -- --| SDB(0) |----|--|-- -- -- -- -- -- -- --\
\033[21;1HBY(0)  |----|--|-- -- -- -- -- -- -- --| SDB(1) |----|--|-- -- -- -- -- -- -- --\
\033[22;1HBY(1)  |----|--|-- -- -- -- -- -- -- --| SDB(2) |----|--|-- -- -- -- -- -- -- --\
\033[23;1HPSU    |----|--|-- -- -- -- -- -- -- --| SDB(3) |----|--|-- -- -- -- -- -- -- --\
\033[24;1HGEND   |----|--|-- -- -- -- -- -- -- --| GENS   |----|--|-- -- -- -- -- -- -- --");
        for (;;) {
                taskDelay(sysClkRateGet() / 20);
                for (i = 0; i < 2; i++) {
                        while (sizeof(can) == rngBufGet(rng_dbg[i], (char *)&can, sizeof(can))) {
                                switch (can.src) {
                                case J1939_ADDR_VSLF:
                                        printf("\033[1;9H%04X", ++total[0]);
                                        break;
                                case J1939_ADDR_VSLB:
                                        printf("\033[2;9H%04X", ++total[1]);
                                        break;
                                case J1939_ADDR_SWH0:
                                        printf("\033[3;9H%04X", ++total[2]);
                                        break;
                                case J1939_ADDR_SWH1:
                                        printf("\033[4;9H%04X", ++total[3]);
                                        break;
                                case J1939_ADDR_SWH2:
                                        printf("\033[5;9H%04X", ++total[4]);
                                        break;
                                case J1939_ADDR_SWH3:
                                        printf("\033[6;9H%04X", ++total[5]);
                                        break;
                                case J1939_ADDR_RSE0:
                                        printf("\033[7;9H%04X", ++total[6]);
                                        break;
                                case J1939_ADDR_RSE1:
                                        printf("\033[8;9H%04X", ++total[7]);
                                        break;
                                case J1939_ADDR_RSE2:
                                        printf("\033[9;9H%04X", ++total[8]);
                                        break;
                                case J1939_ADDR_RSE3:
                                        printf("\033[10;9H%04X", ++total[9]);
                                        break;
                                case J1939_ADDR_SWV0:
                                        printf("\033[11;9H%04X", ++total[10]);
                                        break;
                                case J1939_ADDR_SWV1:
                                        printf("\033[12;9H%04X", ++total[11]);
                                        break;
                                case J1939_ADDR_SWV2:
                                        printf("\033[13;9H%04X", ++total[12]);
                                        break;
                                case J1939_ADDR_SWV3:
                                        printf("\033[14;9H%04X", ++total[13]);
                                        break;
                                case J1939_ADDR_PRP0:
                                        printf("\033[15;9H%04X", ++total[14]);
                                        break;
                                case J1939_ADDR_PRP1:
                                        printf("\033[16;9H%04X", ++total[15]);
                                        break;
                                case J1939_ADDR_PRP2:
                                        printf("\033[17;9H%04X", ++total[16]);
                                        break;
                                case J1939_ADDR_PRP3:
                                        printf("\033[18;9H%04X", ++total[17]);
                                        break;
                                case J1939_ADDR_FY0:
                                        printf("\033[19;50H%04X", ++total[18]);
                                        break;
                                case J1939_ADDR_FY1:
                                        printf("\033[20;50H%04X", ++total[19]);
                                        break;
                                case J1939_ADDR_BY0:
                                        printf("\033[21;50H%04X", ++total[20]);
                                        break;
                                case J1939_ADDR_BY1:
                                        printf("\033[22;50H%04X", ++total[21]);
                                        break;
                                case J1939_ADDR_PSU:
                                        printf("\033[23;9H%04X", ++total[22]);
                                        break;
                                case J1939_ADDR_GEND:
                                        printf("\033[24;9H%04X", ++total[23]);
                                        break;
                                case J1939_ADDR_LVL0:
                                        printf("\033[1;50H%04X", ++total[24]);
                                        break;
                                case J1939_ADDR_LVL1:
                                        printf("\033[2;50H%04X", ++total[25]);
                                        break;
                                case J1939_ADDR_FX:
                                        printf("\033[3;50H%04X", ++total[26]);
                                        break;
                                case J1939_ADDR_BX:
                                        printf("\033[4;50H%04X", ++total[27]);
                                        break;
                                case J1939_ADDR_FZ:
                                        printf("\033[5;50H%04X", ++total[28]);
                                        break;
                                case J1939_ADDR_BZ:
                                        printf("\033[6;50H%04X", ++total[29]);
                                        break;
                                case J1939_ADDR_MOM0:
                                        printf("\033[7;50H%04X", ++total[30]);
                                        break;
                                case J1939_ADDR_MOM1:
                                        printf("\033[8;50H%04X", ++total[31]);
                                        break;
                                case J1939_ADDR_MOM2:
                                        printf("\033[9;50H%04X", ++total[32]);
                                        break;
                                case J1939_ADDR_MOM3:
                                        printf("\033[10;50H%04X", ++total[33]);
                                        break;
                                case J1939_ADDR_TOP:
                                        printf("\033[11;50H%04X", ++total[34]);
                                        break;
                                case J1939_ADDR_SDS0:
                                        printf("\033[12;50H%04X", ++total[35]);
                                        break;
                                case J1939_ADDR_SDS1:
                                        printf("\033[13;50H%04X", ++total[36]);
                                        break;
                                case J1939_ADDR_SDS2:
                                        printf("\033[14;50H%04X", ++total[37]);
                                        break;
                                case J1939_ADDR_SDS3:
                                        printf("\033[15;50H%04X", ++total[38]);
                                        break;
                                case J1939_ADDR_SDF0:
                                        printf("\033[16;50H%04X", ++total[39]);
                                        break;
                                case J1939_ADDR_SDF1:
                                        printf("\033[17;50H%04X", ++total[40]);
                                        break;
                                case J1939_ADDR_SDF2:
                                        printf("\033[18;50H%04X", ++total[41]);
                                        break;
                                case J1939_ADDR_SDF3:
                                        printf("\033[19;50H%04X", ++total[42]);
                                        break;
                                case J1939_ADDR_SDB0:
                                        printf("\033[20;50H%04X", ++total[43]);
                                        break;
                                case J1939_ADDR_SDB1:
                                        printf("\033[21;50H%04X", ++total[44]);
                                        break;
                                case J1939_ADDR_SDB2:
                                        printf("\033[22;50H%04X", ++total[45]);
                                        break;
                                case J1939_ADDR_SDB3:
                                        printf("\033[23;50H%04X", ++total[46]);
                                        break;
                                case J1939_ADDR_GENS:
                                        printf("\033[24;50H%04X", ++total[47]);
                                        break;
                                default:
                                        break;
                                }
                                printf("|%02x|%02x %02x %02x %02x %02x %02x %02x %02x", can.form, can.data[0], can.data[1], can.data[2], can.data[3], can.data[4], can.data[5], can.data[6], can.data[7]);
                        }
                }
        }
}
