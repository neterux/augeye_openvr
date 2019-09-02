#include <winsock2.h>
#include <string>
#include <iostream>

class udp
{
    WSAData wsaData;
    SOCKET sock;
    struct sockaddr_in addr;

public:
    udp(std::string address, int port)
    {
        if (WSAStartup(MAKEWORD(2, 0), &wsaData) != 0)
            std::cout << "WinSock init failed" << std::endl;

        sock = socket(AF_INET, SOCK_DGRAM, 0);

        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = inet_addr(address.c_str());
        addr.sin_port = htons(port);
    }

    ~udp()
    {
        closesocket(sock);
        WSACleanup();
    }

    void udp_send(std::string word)
    {
        sendto(sock, word.c_str(), word.length(), 0, (struct sockaddr*)&addr, sizeof(addr));
    }

    void udp_bind()
    {
        bind(sock, (const struct sockaddr*)&addr, sizeof(addr));
    }

    std::string udp_recv()
    {
#define BUFFER_MAX 400
        char buf[BUFFER_MAX];
        memset(buf, 0, sizeof(buf));
        recv(sock, buf, sizeof(buf), 0);
        return std::string(buf);
    }

    void udp_recv(char* buf, int size)
    {
        memset(buf, 0, size);
        recv(sock, buf, size, 0);
    }
};
