#include <iostream>

// https://www.rsdn.org/article/unix/sockets.xml
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>

#include "netconfig.hpp"

class SocketServer{
public:
    SocketServer(const char* address, int port) : port_(port), address_(address) {
        std::cout << "Init server socket." << "\nIP-address: " << address << "\nPort: " << port << std::endl;

        sock_id = socket(AF_INET, SOCK_STREAM, 0);
        if (sock_id < 0) {
            perror("socket");
            exit(1);
        }

        struct sockaddr_in server_addr;
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(port_);
        server_addr.sin_addr.s_addr = inet_addr(address_);

        if( bind(sock_id, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
            perror("bind");
            exit(2);
        }

        std::cout << "Socket inititialized." << std::endl;
    };

    ~SocketServer(){
        std::cout << "Stop socket." << std::endl;
    };

    int getID() const {
        return sock_id;
    };
private:
    int sock_id;
    const int port_;
    const char* address_;
};

int main(int argc, char *argv[]){
    
    SocketServer server(TARGET_ADDR, TARGET_PORT);
    
    listen(server.getID(), 1);
    
    int sock;
    char buf[1024];
    int bytes_read;

    while(1) {
        sock = accept(server.getID(), NULL, NULL);
        if(sock < 0)
        {
            perror("accept");
            exit(3);
        }

        while(1)
        {
            bytes_read = recv(sock, buf, 1024, 0);
            if(bytes_read <= 0) break;
            std::cout << buf << std::endl;
            send(sock, buf, bytes_read, 0);
        }
    
        close(sock);
    }

    return 0;
}