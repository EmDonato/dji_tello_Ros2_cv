#pragma once
#include <string>
#include <cstdint>
#include <chrono>
#include <sys/socket.h>

#define KEEPALIVEMSG "command"
#define TIME_OUT_RECV_MS 5000 

class UDP_socket_tello
{
private:
    int domain_;
    int type_;
    int protocol_;
    int sock_ = -1;

public:
    UDP_socket_tello(int domain = AF_INET, int type = SOCK_DGRAM, int protocol = 0);
    ~UDP_socket_tello();

    bool open();      
    void close();    
    bool isOpen() const { return sock_ >= 0; }

    bool bind(const std::string& ip, uint16_t port);     
    bool connect(const std::string& ip, uint16_t port);  
    
    int  send(const void* buf, size_t len);  
    int  recv(void* buf, size_t len);        
    bool setRecvTimeout(int ms);
    int keepAlive();
};
