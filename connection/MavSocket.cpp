//
//  mavlink_udp.cpp
//  Drone
//
//  Created by kangzhiyong on 2020/3/1.
//  Copyright © 2020 kangzhiyong. All rights reserved.
//

#ifndef WIN32
#include <sys/errno.h>
#endif // !WIN32


#include "MavSocket.hpp"
#include "DroneUtils.hpp"
#include <time.h>
#include <iostream>
using namespace std;

static void fillAddr(const std::string& address, unsigned short port, sockaddr_in& addr)
{
    memset(&addr, 0, sizeof(addr));  // Zero out address structure
    addr.sin_family = PF_INET;       // Internet address

  //  hostent *host;  // Resolve name
  //  if ((host = gethostbyname(address.c_str())) == NULL) {
  //    // strerror() will not work for gethostbyname() and hstrerror()
  //    // is supposedly obsolete
  //    perror("Failed to resolve name (gethostbyname())");
  //  }
    addr.sin_addr.s_addr = inet_addr(address.c_str());

    addr.sin_port = htons(port);     // Assign port in network byte order
}

int MavSocket::get_socket_fd()
{
    return _socket;
}

MavSocket::MavSocket(int type, int protocol)
{
#ifdef WIN32
    if (!initialized) {
        WORD wVersionRequested;
        WSADATA wsaData;

        wVersionRequested = MAKEWORD(2, 0);              // Request WinSock v2.0
        if (WSAStartup(wVersionRequested, &wsaData) != 0) {  // Load WinSock DLL
            printf("Unable to load WinSock DLL");
        }
        initialized = true;
    }
#endif

    // Make a new socket
    if ((_socket = socket(PF_INET, type, protocol)) < 0) {
        printf("Socket creation failed (socket())");
    }
}

MavSocket::~MavSocket() {
#ifdef WIN32
    ::closesocket(_socket);
#else
    ::close(_socket);
#endif
    _socket = -1;
}

bool MavSocket::recv_msg(mavlink_message_t* msg)
{
    //message receive routine for UDP link
    unsigned char buf[MAX_PACKET_SIZE];
    memset(buf, 0, MAX_PACKET_SIZE);
    int n = recv(buf, MAX_PACKET_SIZE);
    if (n > 0)
    {
        mavlink_status_t status;
        for (unsigned int i = 0; i < n; ++i)
        {
            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], msg, &status))
            {
                // Packet received
                //printf("Received packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", msg->sysid, msg->compid, msg->len, msg->msgid);
                return true;
            }
        }
    }
    return false;
}

bool MavSocket::recv_match(void* msg, bool blocking, int timeout)
{
    /*
    recv the next MAVLink message that matches the given condition
    type can be a string or a list of strings
     */
    time_t start_time = time(nullptr);
    while (1)
    {
        //        if (timeout != 0)
        //        {
        //            time_t now = time(nullptr);
        //            if (now < start_time)
        //            {
        //                start_time = now; // If an external process rolls back system time, we should not spin forever.
        //            }
        //            if (start_time + timeout < time(nullptr))
        //            {
        //                return false;
        //            }
        //        }
        if (!recv_msg((mavlink_message_t*)msg))
        {
            if (blocking)
            {
                sleep((unsigned int)timeout);
                continue;
            }
            return false;
        }
        else
        {
            return true;
        }
    }
    return false;
}

MavUDP::MavUDP(std::string dest_ip, unsigned short dest_port, bool input, bool broadcast, int source_system, int source_component, bool use_native): MavSocket(SOCK_DGRAM, IPPROTO_UDP){
    int val = 1;
    destination_addr = dest_ip;
    destination_port = dest_port;
    setsockopt(_socket, SOL_SOCKET, SO_REUSEADDR, (char*)&val, sizeof(val));
    struct ip_mreq mreq;
    mreq.imr_multiaddr.s_addr = inet_addr("224.0.0.1");
    mreq.imr_interface.s_addr = htonl(INADDR_ANY);
    if (setsockopt(_socket, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*)&mreq, sizeof(mreq)) < 0)
    {
        perror("setsockopt MEMBERSHIP");
        return;
    }
    sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));  // Zero out address structure
    addr.sin_family = PF_INET;       // Internet address
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(LOCAL_PORT);     // Assign port in network byte order

    if (::bind(_socket, (sockaddr *) &addr, sizeof(sockaddr_in)) < 0)
    {
        perror("bind:");
        return;
    }

    last_addr = "";
    last_port = 0;
    resolved_destination_addr = "";
    resolved_destination_port = 0;
    _connected = true;
}

int MavUDP::recv(void *buffer, int bufferLen)
{
    sockaddr_in clntAddr;
    socklen_t addrLen = sizeof(clntAddr);
    ssize_t rtn;
    if ((rtn = ::recvfrom(_socket, (char*)buffer, bufferLen, 0, (struct sockaddr *)&clntAddr, (socklen_t *) &addrLen)) < 0) {
        if (errno != EAGAIN)
        {
            perror("Receive failed (recvfrom()): ");
            return -1;
        }
    }
//    destination_addr = inet_ntoa(clntAddr.sin_addr);
//    destination_port = ntohs(clntAddr.sin_port);
    return (int)rtn;
}

void MavUDP::write(const void *buffer, int bufferLen)
{
    sockaddr_in destAddr;

    if (udp_server)
    {
        if (last_addr != "" && last_port != 0) {
            fillAddr(last_addr, last_port, destAddr);
            if (::sendto(_socket, (char*)buffer, bufferLen, 0, (struct sockaddr *)&destAddr, sizeof(destAddr)) != bufferLen)
            {
                perror("Send failed (sendto()): ");
            }
        }
    }
    else
    {
        if (last_addr != "" && _broadcast)
        {
            destination_addr = last_addr;
            destination_port = last_port;
            _broadcast = false;
            fillAddr(destination_addr, destination_port, destAddr);
            ::connect(_socket, (struct sockaddr *)&destAddr, sizeof(destAddr));
        }
        if (!destination_addr.empty() && destination_port > 0) {
            fillAddr(destination_addr, destination_port, destAddr);
            // Write out the whole buffer as a single message.
            if (sendto(_socket, (char*)buffer, bufferLen, 0, (sockaddr *) &destAddr, sizeof(destAddr)) != bufferLen) {
                perror("Send failed (sendto()): ");
            }
        }
    }
}

MavTCP::MavTCP(std::string dest_ip, unsigned short  dest_port, bool input, bool broadcast, int source_system, int source_component, bool use_native): MavSocket(SOCK_STREAM, IPPROTO_TCP)
{
    int val = 1;
    setsockopt(_socket, SOL_SOCKET, SO_REUSEADDR, (char*)&val, sizeof(val));

    sockaddr_in destAddr;
    fillAddr(dest_ip, dest_port, destAddr);

    // Try to connect to the given port
    if (::connect(_socket, (sockaddr*)&destAddr, sizeof(destAddr)) < 0) {
        printf("Connect failed (connect())\r\n");
    }
}

void MavTCP::write(const void* buffer, int bufferLen)
{
    if (::send(_socket, (raw_type*)buffer, bufferLen, 0) < 0) {
        printf("Send failed (send())");
    }
}

int MavTCP::recv(void* buffer, int bufferLen)
{
    ssize_t rtn;
    if ((rtn = ::recv(_socket, (raw_type*)buffer, bufferLen, 0)) < 0) {
        cout << "Received failed (recv()):" << strerror(errno) << endl;
    }

    return (int)rtn;
}
