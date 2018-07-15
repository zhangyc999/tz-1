#include "j1939.h"
#include "struct.h"
#include "type.h"
#include "vx.h"

#define SERVER_ADDRESS "192.168.100.130"
#define SERVER_PORT 4207
#define CLIENT_PORT 4201
#define GROUP_ADDRESS "234.1.1.9"

void t_udp_rx(int fd);
void t_udp_tx(int fd);
int remap_addr_index(u8 addr);
u8 check_xor(u8 *buf, int n);

extern MSG_Q_ID msg_main;
extern RING_ID rng_udp[];
extern RING_ID rng_result;

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
                taskDelay(20);
                while (ERROR != recvfrom(fd, (char *)&rx, sizeof(rx), 0, (struct sockaddr *)&server, &size))
                        msgQSend(msg_main, (char *)&rx.cmd, sizeof(rx.cmd), NO_WAIT, MSG_PRI_NORMAL);
        }
}

void t_udp_tx(int fd)
{
        struct frame_udp_tx tx;
        struct frame_can can;
        struct sockaddr_in client;
        int size = sizeof(struct sockaddr_in);
        int offset = -1;
        int result[2] = {0, 0};
        int i = 0;
        client.sin_len = (u_char)size;
        client.sin_family = AF_INET;
        client.sin_port = htons(CLIENT_PORT);
        client.sin_addr.s_addr = inet_addr(GROUP_ADDRESS);
        bzero((char *)&tx, sizeof(tx));
        tx.head = 0xC7FEC7FE;
        for (;;) {
                taskDelay(20);
                for (i = 0; i < 2; i++) {
                        while (sizeof(can) == rngBufGet(rng_udp[i], (char *)&can, sizeof(can))) {
                                offset = remap_addr_index(can.src);
                                if (offset >= 0 && offset < 50)
                                        memcpy((u8 *)&tx + 8 + offset * (sizeof(can) - 4), &can, sizeof(can) - 4);
                        }
                }
                while (sizeof(result) == rngBufGet(rng_result, (char *)result, sizeof(result))) {
                        offset = remap_addr_index(result[0]);
                        if (offset >= 0 && offset < 50)
                                memcpy((u8 *)&tx + 8 + 50 * (sizeof(can) - 4) + offset * sizeof(int), &result[1], sizeof(int));
                }
                sendto(fd, (caddr_t)&tx, sizeof(tx), 0, (struct sockaddr *)&client, size);
        }
}

u8 check_xor(u8 *buf, int n)
{
        if (n > 1)
                return *buf ^ check_xor(buf + 1, n - 1);
        return *buf;
}
