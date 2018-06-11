#ifndef DEFINE_H_
#define DEFINE_H_

#define UNMASK_TASK_NOTIFY 0xFFFF0000
#define TASK_NOTIFY_PWR    0x5A020000
#define TASK_NOTIFY_SWH    0x5A100000

#define UNMASK_TASK_STATE 0x0000FFFF
#define TASK_STATE_FAULT  0x000000E0
#define TASK_STATE_OK     0x000000D7
#define TASK_STATE_SAFE   0x0000C300
#define TASK_STATE_DANGER 0x0000A900

#define UNMASK_CMD_ACT   0xFF00F00F
#define UNMASK_CMD_MODE  0x00F00000
#define UNMASK_CMD_DIR   0x000F0000
#define CMD_IDLE         0x00000000
#define CMD_ACT_GEND     0xEC001001
#define CMD_ACT_GENS     0xEC001002
#define CMD_ACT_PSU      0xEC001003
#define CMD_ACT_SWH      0xEC002001
#define CMD_MODE_AUTO    0x00000000
#define CMD_MODE_MANUAL  0x00E00000
#define CMD_DIR_POSI     0x00000000
#define CMD_DIR_NEGA     0x000C0000
#define CMD_DIR_STOP     0x000A0000

#endif /* DEFINE_H_ */
