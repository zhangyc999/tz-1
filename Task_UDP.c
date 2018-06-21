#include "j1939.h"
#include "struct.h"
#include "type.h"
#include "vx.h"

#define SERVER_ADDRESS "192.168.100.130"
#define SERVER_PORT 4207
#define CLIENT_PORT 4201
#define GROUP_ADDRESS "234.1.1.9"

extern MSG_Q_ID msg_udp;
extern MSG_Q_ID msg_main;

void t_udp_rx(int fd);
void t_udp_tx(int fd);
u8 check_xor(u8 *buf, int n);

void udp_server(void)
{
        struct sockaddr_in server;
        struct ip_mreq group;
        int size = sizeof(struct sockaddr_in);
        int fd = socket(AF_INET, SOCK_DGRAM, 0);
        int offset = 0;
        /* u_long mode = 1; */
        bzero((char *)&server, size);
        server.sin_len = (u_char)size;
        server.sin_family = AF_INET;
        server.sin_port = htons(SERVER_PORT);
        server.sin_addr.s_addr = htonl(INADDR_ANY);
        /* ioctl(fd, FIONBIO, (int)&mode); */
        bind(fd, (struct sockaddr *)&server, size);
        group.imr_multiaddr.s_addr = inet_addr(GROUP_ADDRESS);
        group.imr_interface.s_addr = inet_addr(SERVER_ADDRESS);
        routeAdd(GROUP_ADDRESS, SERVER_ADDRESS);
        setsockopt(fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *)&group, sizeof(group));
        taskSpawn("UDP_RX", 90, VX_FP_TASK, 20000, (FUNCPTR)t_udp_rx, fd, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        taskSpawn("UDP_TX", 90, VX_FP_TASK, 20000, (FUNCPTR)t_udp_tx, fd, 0, 0, 0, 0, 0, 0, 0, 0, 0);
}

void t_udp_rx(int fd)
{
        struct frame_udp_rx rx;
        struct sockaddr_in server;
        int size = sizeof(struct sockaddr_in);
        for (;;) {
                if (ERROR != recvfrom(fd, (char *)&rx, sizeof(rx), 0, (struct sockaddr *)&server, &size))
                        msgQSend(msg_main, (char *)&rx.cmd, sizeof(rx.cmd), NO_WAIT, MSG_PRI_NORMAL);
                taskDelay(20);
        }
}

void t_udp_tx(int fd)
{
        struct frame_udp_tx tx;
        struct frame_can can;
        struct sockaddr_in client;
        int size = sizeof(struct sockaddr_in);
        int offset = -1;
        client.sin_len = (u_char)size;
        client.sin_family = AF_INET;
        client.sin_port = htons(CLIENT_PORT);
        client.sin_addr.s_addr = inet_addr(GROUP_ADDRESS);
        bzero((char *)&tx, sizeof(tx));
        tx.head = 0xC7FEC7FE;
        for (;;) {
                while (msgQNumMsgs(msg_udp) > 0) {
                        msgQReceive(msg_udp, (char *)&can, sizeof(can), NO_WAIT);
                        switch (can.src) {
                        case J1939_ADDR_VSLF:
                                offset = 0;
                                break;
                        case J1939_ADDR_VSLB:
                                offset = 1;
                                break;
                        case J1939_ADDR_LVL0:
                                offset = 2;
                                break;
                        case J1939_ADDR_LVL1:
                                offset = 3;
                                break;
                        case J1939_ADDR_FZ:
                                offset = 4;
                                break;
                        case J1939_ADDR_BZ:
                                offset = 5;
                                break;
                        case J1939_ADDR_FY0:
                                offset = 6;
                                break;
                        case J1939_ADDR_FY1:
                                offset = 7;
                                break;
                        case J1939_ADDR_BY0:
                                offset = 8;
                                break;
                        case J1939_ADDR_BY1:
                                offset = 9;
                                break;
                        case J1939_ADDR_FX:
                                offset = 10;
                                break;
                        case J1939_ADDR_BX:
                                offset = 11;
                                break;
                        case J1939_ADDR_PRP0:
                                offset = 12;
                                break;
                        case J1939_ADDR_PRP1:
                                offset = 13;
                                break;
                        case J1939_ADDR_PRP2:
                                offset = 14;
                                break;
                        case J1939_ADDR_PRP3:
                                offset = 15;
                                break;
                        case J1939_ADDR_RSE0:
                                offset = 16;
                                break;
                        case J1939_ADDR_RSE1:
                                offset = 17;
                                break;
                        case J1939_ADDR_RSE2:
                                offset = 18;
                                break;
                        case J1939_ADDR_RSE3:
                                offset = 19;
                                break;
                        case J1939_ADDR_SWV0:
                                offset = 20;
                                break;
                        case J1939_ADDR_SWV1:
                                offset = 21;
                                break;
                        case J1939_ADDR_SWV2:
                                offset = 22;
                                break;
                        case J1939_ADDR_SWV3:
                                offset = 23;
                                break;
                        case J1939_ADDR_SWH0:
                                offset = 24;
                                break;
                        case J1939_ADDR_SWH1:
                                offset = 25;
                                break;
                        case J1939_ADDR_SWH2:
                                offset = 26;
                                break;
                        case J1939_ADDR_SWH3:
                                offset = 27;
                                break;
                        case J1939_ADDR_GEND:
                                offset = 28;
                                break;
                        case J1939_ADDR_GENS:
                                offset = 29;
                                break;
                        case J1939_ADDR_PSU:
                                offset = 30;
                                break;
                        case J1939_ADDR_MOM0:
                                offset = 32;
                                break;
                        case J1939_ADDR_MOM1:
                                offset = 33;
                                break;
                        case J1939_ADDR_MOM2:
                                offset = 34;
                                break;
                        case J1939_ADDR_MOM3:
                                offset = 35;
                                break;
                        case J1939_ADDR_SDT:
                                offset = 36;
                                break;
                        case J1939_ADDR_SDS0:
                                offset = 37;
                                break;
                        case J1939_ADDR_SDS1:
                                offset = 38;
                                break;
                        case J1939_ADDR_SDS2:
                                offset = 39;
                                break;
                        case J1939_ADDR_SDS3:
                                offset = 40;
                                break;
                        case J1939_ADDR_SDF0:
                                offset = 41;
                                break;
                        case J1939_ADDR_SDF1:
                                offset = 42;
                                break;
                        case J1939_ADDR_SDF2:
                                offset = 43;
                                break;
                        case J1939_ADDR_SDF3:
                                offset = 44;
                                break;
                        case J1939_ADDR_SDB0:
                                offset = 45;
                                break;
                        case J1939_ADDR_SDB1:
                                offset = 46;
                                break;
                        case J1939_ADDR_SDB2:
                                offset = 47;
                                break;
                        case J1939_ADDR_SDB3:
                                offset = 48;
                                break;
                        default:
                                break;
                        }
                        if (offset >= 0 && offset < 50)
                                memcpy((u8 *)&tx + 8 + offset * (sizeof(can) - 4), &can, sizeof(can) - 4);
                }
                sendto(fd, (caddr_t)&tx, sizeof(tx), 0, (struct sockaddr *)&client, size);
                taskDelay(20);
        }
}

u8 check_xor(u8 *buf, int n)
{
        if (n > 1)
                return *buf ^ check_xor(buf + 1, n - 1);
        return *buf;
}
