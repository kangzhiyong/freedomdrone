#pragma once

#ifdef WIN32
//#include <Ws2def.h>
  //#include <winsock.h>         // For socket(), connect(), send(), and recv()
#include <winsock2.h> 
#include <ws2tcpip.h>
#include <mswsock.h>
typedef int socklen_t;
typedef char raw_type;       // Type used for raw data on this platform
//typedef uint32_t in_addr_t;
#define ssize_t size_t
#pragma comment(lib,"ws2_32.lib")
#else
#include <sys/types.h>       // For data types
#include <sys/socket.h>      // For socket(), connect(), send(), and recv()
#include <netdb.h>           // For gethostbyname()
#include <arpa/inet.h>       // For inet_addr()
#include <unistd.h>          // For close()
#include <netinet/in.h>      // For sockaddr_in
typedef void raw_type;       // Type used for raw data on this platform
#endif

#include <string>

#ifdef WIN32
static bool initialized = false;
#endif

#include "mavlink/common/mavlink.h"

#define MAX_PACKET_SIZE 65467 // UDP protocol max message size
#define LOCAL_PORT  14550
#define DEST_PORT 14555
#define DEST_IP "192.168.4.1"

class MavSocket
{
public:
    MavSocket(int type, int protocol);
    ~MavSocket();
    virtual int recv(void* buffer, int bufferLen) = 0;
    bool recv_msg(mavlink_message_t* msg);
    bool recv_match(void* msg, bool blocking = false, int timeout = 0);
    virtual void write(const void* buffer, int bufferLen) = 0;
    int get_socket_fd();
protected:
    int _socket{ -1 };
};

class MavUDP: public MavSocket
{
    //a UDP mavlink socket
public:
    MavUDP(std::string dest_ip, unsigned short dest_port, bool input=true, bool broadcast=false, int source_system=255, int source_component=0, bool use_native=true);
    int recv(void *buffer, int bufferLen);
    void write(const void *buffer, int bufferLen);
    bool connected()
    {
        return _connected;
    }
private:
    bool udp_server{false};
    bool _broadcast{ false };
    std::string destination_addr{ DEST_IP };
    int destination_port{ DEST_PORT };
    std::string last_addr{ "" };
    int last_port{ -1 };
    std::string resolved_destination_addr{ "" };
    int resolved_destination_port{ -1 };
    bool _connected{ false };
};

class MavTCP: public MavSocket
{
    //a TCP mavlink socket
public:
    MavTCP(std::string dest_ip, unsigned short dest_port, bool input = true, bool broadcast = false, int source_system = 255, int source_component = 0, bool use_native = true);
    ~MavTCP();
    int recv(void* buffer, int bufferLen);
    void write(const void* buffer, int bufferLen);
private:
    bool tcp_server{ false };
};
