#ifndef CYLINDER_H_
#define CYLINDER_H_

#define cyl_cmd(x) do {                                  \
        cmd = *(struct main *)tmp;                       \
        switch (verify.type) {                           \
        case CMD_IDLE:                                   \
        case x | CMD_MODE_AUTO | CMD_DIR_STOP:           \
        case x | CMD_MODE_MANUAL | CMD_DIR_STOP:         \
                switch (cmd.type) {                      \
                case CMD_IDLE:                           \
                case x | CMD_MODE_AUTO | CMD_DIR_POSI:   \
                case x | CMD_MODE_AUTO | CMD_DIR_NEGA:   \
                case x | CMD_MODE_AUTO | CMD_DIR_STOP:   \
                case x | CMD_MODE_MANUAL | CMD_DIR_POSI: \
                case x | CMD_MODE_MANUAL | CMD_DIR_NEGA: \
                case x | CMD_MODE_MANUAL | CMD_DIR_STOP: \
                        verify = cmd;                    \
                        break;                           \
                default:                                 \
                        break;                           \
                }                                        \
                break;                                   \
        case x | CMD_MODE_AUTO | CMD_DIR_POSI:           \
                switch (cmd.type) {                      \
                case x | CMD_MODE_AUTO | CMD_DIR_POSI:   \
                case x | CMD_MODE_AUTO | CMD_DIR_STOP:   \
                case x | CMD_MODE_MANUAL | CMD_DIR_STOP: \
                        verify = cmd;                    \
                        break;                           \
                default:                                 \
                        break;                           \
                }                                        \
                break;                                   \
        case x | CMD_MODE_AUTO | CMD_DIR_NEGA:           \
                switch (cmd.type) {                      \
                case x | CMD_MODE_AUTO | CMD_DIR_NEGA:   \
                case x | CMD_MODE_AUTO | CMD_DIR_STOP:   \
                case x | CMD_MODE_MANUAL | CMD_DIR_STOP: \
                        verify = cmd;                    \
                        break;                           \
                default:                                 \
                        break;                           \
                }                                        \
                break;                                   \
        case x | CMD_MODE_MANUAL | CMD_DIR_POSI:         \
                switch (cmd.type) {                      \
                case x | CMD_MODE_AUTO | CMD_DIR_STOP:   \
                case x | CMD_MODE_MANUAL | CMD_DIR_POSI: \
                case x | CMD_MODE_MANUAL | CMD_DIR_STOP: \
                        verify = cmd;                    \
                        break;                           \
                default:                                 \
                        break;                           \
                }                                        \
                break;                                   \
        case x | CMD_MODE_MANUAL | CMD_DIR_NEGA:         \
                switch (cmd.type) {                      \
                case x | CMD_MODE_AUTO | CMD_DIR_STOP:   \
                case x | CMD_MODE_MANUAL | CMD_DIR_NEGA: \
                case x | CMD_MODE_MANUAL | CMD_DIR_STOP: \
                        verify = cmd;                    \
                        break;                           \
                default:                                 \
                        break;                           \
                }                                        \
                break;                                   \
        default:                                         \
                break;                                   \
        }                                                \
} while(0)

#endif /* CYLINDER_H_ */
