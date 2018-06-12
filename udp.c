#include "struct.h"
#include "type.h"
#include "vx.h"

#define SERVER_ADDRESS "192.168.100.130"
#define SERVER_PORT 4207
#define CLIENT_PORT 4201
#define GROUP_ADDRESS "234.1.1.9"

extern MSG_Q_ID msg_main;

void t_udp(void)
{
        static char msg[] = {'\xFE', '\xC7', '\x13', '\xF7', '\x11', '\x22', '\x33', '\x44'};
        struct frame_udp_rx rx;
        struct sockaddr_in server;
        struct sockaddr_in client;
        struct ip_mreq group;
        int size = sizeof(struct sockaddr_in);
        int fd = socket(AF_INET, SOCK_DGRAM, 0);
        u_long mode = 1;
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
        ioctl(fd, FIONBIO, (int)&mode);
        bind(fd, (struct sockaddr *)&server, size);
        group.imr_multiaddr.s_addr = inet_addr(GROUP_ADDRESS);
        group.imr_interface.s_addr = inet_addr(SERVER_ADDRESS);
        routeAdd(GROUP_ADDRESS, SERVER_ADDRESS);
        setsockopt(fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *)&group, sizeof(group));
        taskDelay(20);
        for (;;) {
                sendto(fd, msg, sizeof(msg), 0, (struct sockaddr *)&client, size);
                if (ERROR != recvfrom(fd, (char *)&rx, sizeof(rx), 0, (struct sockaddr *)&client, &size))
                        msgQSend(msg_main, (char *)&rx.cmd, sizeof(rx.cmd), NO_WAIT, MSG_PRI_NORMAL);
                taskDelay(20);
        }
}
