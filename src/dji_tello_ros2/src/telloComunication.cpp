
#include "dji_tello_Ros2/telloComunication.hpp"

#include <cerrno>
#include <cstring>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>



static bool toSockAddrIPv4(const std::string& ip, uint16_t port, sockaddr_in& out)
{
    std::memset(&out, 0, sizeof(out));
    out.sin_family = AF_INET;
    out.sin_port   = htons(port);
    return ::inet_pton(AF_INET, ip.c_str(), &out.sin_addr) == 1;
}



UDP_socket_tello::UDP_socket_tello(int d, int t, int p)
: domain_(d), type_(t), protocol_(p) {}

UDP_socket_tello::~UDP_socket_tello() { close(); }

bool UDP_socket_tello::open()
{
    close(); 
    sock_ = ::socket(domain_, type_, protocol_);
    if (sock_ >= 0) {

        int yes = 1;
        ::setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
        return true;

    }
    return false;
}

void UDP_socket_tello::close() {
    if (sock_ >= 0) { ::close(sock_); sock_ = -1; }
}

bool UDP_socket_tello::bind(const std::string& ip, uint16_t port) {
    if (!isOpen() && !open()) return false;
    sockaddr_in a{};
    if (!toSockAddrIPv4(ip, port, a)) return false;
    setRecvTimeout(TIME_OUT_RECV_MS); // 5s
    return ::bind(sock_, reinterpret_cast<sockaddr*>(&a), sizeof(a)) == 0;
}

bool UDP_socket_tello::connect(const std::string& ip, uint16_t port) {
    if (!isOpen() && !open()) return false;
    sockaddr_in a{};
    setRecvTimeout(TIME_OUT_RECV_MS); // 5s
    if (!toSockAddrIPv4(ip, port, a)) return false;
    return ::connect(sock_, reinterpret_cast<sockaddr*>(&a), sizeof(a)) == 0;
}

int UDP_socket_tello::send(const void* buf, size_t len) {
    if (sock_ < 0) return -1;
    ssize_t n = ::send(sock_, buf, len, 0);
    return (n < 0) ? -1 : static_cast<int>(n);
}

int UDP_socket_tello::recv(void* buf, size_t len) {
    if (sock_ < 0) return -1;
    ssize_t n = ::recv(sock_, buf, len, 0);
    return (n < 0) ? -1 : static_cast<int>(n);
}

bool UDP_socket_tello::setRecvTimeout(int ms) {
    if (sock_ < 0) return false;
    timeval tv{};
    tv.tv_sec  = ms / 1000;
    tv.tv_usec = (ms % 1000) * 1000;
    return ::setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) == 0;
}

int UDP_socket_tello::keepAlive() {
    if (sock_ < 0) return -1;
    ssize_t n = ::send(sock_, KEEPALIVEMSG, std::strlen(KEEPALIVEMSG), 0);
    return (n < 0) ? -1 : static_cast<int>(n);
}

