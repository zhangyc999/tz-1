#include "addr.h"
#include "struct.h"
#include "type.h"
#include "vx.h"

#define SERVER_ADDRESS "192.168.100.130"
#define SERVER_PORT 4207
#define CLIENT_PORT 4201
#define GROUP_ADDRESS "234.1.1.9"

extern MSG_Q_ID msg_udp;
extern MSG_Q_ID msg_main;

u8 check_xor(u8 *buf, int n);

void t_udp(void)
{
        static char replyMsg[] = {'\xFE', '\xC7', '\x13', '\xF7', '\x11', '\x22', '\x33', '\x44'};
        struct frame_udp_rx rx;
        struct frame_udp_tx tx;
        struct frame_can can;
        struct sockaddr_in server;
        struct sockaddr_in client;
        struct sockaddr_in tmp;
        struct ip_mreq group;
        int size = sizeof(struct sockaddr_in);
        int fd = socket(AF_INET, SOCK_DGRAM, 0);
        u_long mode = 1;
        int offset = 0;
        bzero((char *)&server, size);
        server.sin_len = (u_char)size;
        server.sin_family = AF_INET;
        server.sin_port = htons(SERVER_PORT);
        server.sin_addr.s_addr = htonl(INADDR_ANY);
        bzero((char *)&client, size);
        client.sin_len = (u_char)size;
        client.sin_family = AF_INET;
        client.sin_port = htons(CLIENT_PORT);
        client.sin_addr.s_addr = inet_addr(GROUP_ADDRESS);
        /*        ioctl(fd, FIONBIO, (int)&mode);*/
        bind(fd, (struct sockaddr *)&server, size);
        group.imr_multiaddr.s_addr = inet_addr(GROUP_ADDRESS);
        group.imr_interface.s_addr = inet_addr(SERVER_ADDRESS);
        routeAdd(GROUP_ADDRESS, SERVER_ADDRESS);
        setsockopt(fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *)&group, sizeof(group));
        bzero((char *)&tx, sizeof(tx));
        tx.head = 0xC7FEC7FE;
        for (;;) {
#if 0
                msgQReceive(msg_udp, (char *)&can, sizeof(can), NO_WAIT);
                switch (can.src) {
                case ADDR_VSLF:
                        offset = 1;
                        break;
                case ADDR_VSLB:
                        offset = 2;
                        break;
                case ADDR_SWH0:
                        offset = 3;
                        break;
                case ADDR_SWH1:
                        offset = 4;
                        break;
                case ADDR_SWH2:
                        offset = 5;
                        break;
                case ADDR_SWH3:
                        offset = 6;
                        break;
                case ADDR_RSE0:
                        offset = 7;
                        break;
                case ADDR_RSE1:
                        offset = 8;
                        break;
                case ADDR_RSE2:
                        offset = 9;
                        break;
                case ADDR_RSE3:
                        offset = 10;
                        break;
                case ADDR_SWV0:
                        offset = 11;
                        break;
                case ADDR_SWV1:
                        offset = 12;
                        break;
                case ADDR_SWV2:
                        offset = 13;
                        break;
                case ADDR_SWV3:
                        offset = 14;
                        break;
                case ADDR_PRP0:
                        offset = 15;
                        break;
                case ADDR_PRP1:
                        offset = 16;
                        break;
                case ADDR_PRP2:
                        offset = 17;
                        break;
                case ADDR_PRP3:
                        offset = 18;
                        break;
                case ADDR_FY0:
                        offset = 19;
                        break;
                case ADDR_FY1:
                        offset = 20;
                        break;
                case ADDR_BY0:
                        offset = 21;
                        break;
                case ADDR_BY1:
                        offset = 22;
                        break;
                case ADDR_PSU:
                        offset = 23;
                        break;
                case ADDR_GEND:
                        offset = 24;
                        break;
                case ADDR_LVL0:
                        offset = 25;
                        break;
                case ADDR_LVL1:
                        offset = 26;
                        break;
                case ADDR_FX:
                        offset = 27;
                        break;
                case ADDR_BX:
                        offset = 28;
                        break;
                case ADDR_FZ:
                        offset = 29;
                        break;
                case ADDR_BZ:
                        offset = 30;
                        break;
                case ADDR_MOM0:
                        offset = 31;
                        break;
                case ADDR_MOM1:
                        offset = 32;
                        break;
                case ADDR_MOM2:
                        offset = 33;
                        break;
                case ADDR_MOM3:
                        offset = 34;
                        break;
                case ADDR_SDT:
                        offset = 35;
                        break;
                case ADDR_SDS0:
                        offset = 36;
                        break;
                case ADDR_SDS1:
                        offset = 37;
                        break;
                case ADDR_SDS2:
                        offset = 38;
                        break;
                case ADDR_SDS3:
                        offset = 39;
                        break;
                case ADDR_SDF0:
                        offset = 40;
                        break;
                case ADDR_SDF1:
                        offset = 41;
                        break;
                case ADDR_SDF2:
                        offset = 42;
                        break;
                case ADDR_SDF3:
                        offset = 43;
                        break;
                case ADDR_SDB0:
                        offset = 44;
                        break;
                case ADDR_SDB1:
                        offset = 45;
                        break;
                case ADDR_SDB2:
                        offset = 46;
                        break;
                case ADDR_SDB3:
                        offset = 47;
                        break;
                case ADDR_GENS:
                        offset = 48;
                        break;
                default:
                        break;
                }
#endif
                sendto(fd, (caddr_t)&tx, sizeof(tx), 0, (struct sockaddr *)&client, size);
                printf("x");
                if (ERROR != recvfrom(fd, (char *)&rx, sizeof(rx), 0, (struct sockaddr *)&server, &size))
                        msgQSend(msg_main, (char *)&rx.cmd, sizeof(rx.cmd), NO_WAIT, MSG_PRI_NORMAL);
                taskDelay(20);
        }
}

u8 check_xor(u8 *buf, int n)
{
        if (n > 1)
                return *buf ^ check_xor(buf + 1, n - 1);
        return *buf;
}
