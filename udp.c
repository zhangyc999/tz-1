#include "type.h"
#include "vx.h"

#define SERVER_ADDRESS "192.168.100.130"
#define SERVER_PORT 4207
#define CLIENT_PORT 4201
#define GROUP_ADDRESS "234.1.1.9"

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

extern int udp_socket;
extern MSG_Q_ID msg_main;

void t_udp(void)
{
        static char msg[] = {'\xFE', '\xC7', '\x13', '\xF7', '\x11', '\x22', '\x33', '\x44'};
        struct udp_cmd udp;
        struct sockaddr_in server;
        struct sockaddr_in client;
        struct ip_mreq group;
	int size = sizeof(struct sockaddr_in);
        bzero((char *)&server, size);
        server.sin_len = (u8)size;
        server.sin_family = AF_INET;
        server.sin_port = htons(SERVER_PORT);
        server.sin_addr.s_addr = htonl(INADDR_ANY);
	bind(udp_socket, (struct sockaddr *)&server, size);
        bzero((char *)&client, size);
        client.sin_len = (u8)size;
        client.sin_family = AF_INET;
        client.sin_port = htons(CLIENT_PORT);
        client.sin_addr.s_addr = inet_addr(GROUP_ADDRESS);
        group.imr_multiaddr.s_addr = inet_addr(GROUP_ADDRESS);
        group.imr_interface.s_addr = inet_addr(SERVER_ADDRESS);
        routeAdd(GROUP_ADDRESS, SERVER_ADDRESS);
	setsockopt(udp_socket, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *)&group, sizeof(group));
        taskDelay(20);
        for (;;) {
                sendto(udp_socket, msg, sizeof(msg), 0, (struct sockaddr *)&client, size);
                if (ERROR != recvfrom(udp_socket, (char *)&udp, sizeof(udp), 0, (struct sockaddr *)&client, &size))
                        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
                taskDelay(20);
        }
}
