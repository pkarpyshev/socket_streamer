#pragma once

#include <iostream>

// https://www.rsdn.org/article/unix/sockets.xml
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>

#define TARGET_ADDR "10.30.33.32"
#define TARGET_PORT 40000

class SocketServer{
public:
    SocketServer(const char* address, int port) : port_(port), address_(address) {
        std::cout << "Init server socket." << "\nIP-address: " << address << "\nPort: " << port << std::endl;

        sock_id = socket(AF_INET, SOCK_STREAM, 0);
        if (sock_id < 0) {
            perror("socket");
            exit(1);
        }

        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(port_);
        server_addr.sin_addr.s_addr = inet_addr(address_);

        if( bind(sock_id, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
            perror("bind");
            exit(2);
        }

        std::cout << "Server inititialized." << std::endl;
    };

    ~SocketServer(){
        std::cout << "Stop server." << std::endl;
    };

    bool sendRequest(int sockID){
        return (send(sockID, "1", 1, 0) == 1);
    }

    int getID() const {
        return sock_id;
    };

    struct sockaddr_in getSockaddr() const {
        return server_addr;
    };

private:
    int sock_id;
    const int port_;
    const char* address_;
    struct sockaddr_in server_addr;

};

class SocketClient{
public:
    SocketClient(const char* address, int port) : port_(port), address_(address){
        std::cout << "Init client socket." << "\nIP-address: " << address << "\nPort: " << port << std::endl;
        sock_id = socket(AF_INET, SOCK_STREAM, 0);
        if(sock_id < 0)
        {
            perror("socket");
            exit(1);
        }

        client_addr.sin_family = AF_INET;
        client_addr.sin_port = htons(port_);
        client_addr.sin_addr.s_addr = inet_addr(address_);

        if(connect(sock_id, (struct sockaddr *)&client_addr, sizeof(client_addr)) < 0)
        {
            perror("connect");
            exit(2);
        }
        std::cout << "Client inititialized." << std::endl;
    };

    ~SocketClient(){

    };
    
    bool isRequested(){
        if (recv(sock_id, &request, 1, 0)){
            std::cout << "Request: " << request << std::endl;
            return true;
        }
        return false;
    }

    int getID() const {
        return sock_id;
    };

    struct sockaddr_in getSockaddr() const {
        return client_addr;
    };

    char request = '0';
private:
    int sock_id;
    const int port_;
    const char* address_;
    struct sockaddr_in client_addr;

};