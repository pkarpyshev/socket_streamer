#include <iostream>

// https://www.rsdn.org/article/unix/sockets.xml

#include "netconfig.hpp"

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