/******************************************************************
CanBus class for can-bus communication

Features:
- SocketCAN
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2021-07-03: Initial version
2021-08-29: Update declaration based on new coding formatting
2024-01-22: Add extended frame support
2024-xx-xx: xxx
******************************************************************/
#pragma once
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/epoll.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <string>
#include <memory>

class CanBus
{
public:
	using SharedPtr = std::shared_ptr<CanBus>;

public:
	CanBus() = delete;
	CanBus(const char* Name);
	CanBus(const std::string& Name);
	~CanBus() { close(); };

public:
	bool open();
	bool open(const std::string& Name);
	bool isOpen();
	void close();
	ssize_t read(unsigned int& ID, unsigned char* Data);
	bool write(unsigned int ID, size_t Len, const unsigned char* Data);
	std::size_t eventTriggered(unsigned int& ID, unsigned char* Data);
	bool isExtended(unsigned int ID) const;

protected:
	struct IfInfo // bundled information per open socket
	{
		int socket_{ -1 };
		__u32 dropcnt_{ 0 };
		__u32 last_dropcnt_{ 0 };
	};

protected:
	struct sockaddr_can addr_;
	struct ifreq ifr_;
	struct IfInfo if_obj_;
	int fd_epoll_{ -1 };
	bool is_open_{ false };
	struct canfd_frame frame_;
	struct iovec iov_;
	struct msghdr msg_;
	char ctrlmsg[CMSG_SPACE(sizeof(struct timeval) + 3 * sizeof(struct timespec) + sizeof(__u32))];
	struct epoll_event event_pending_;
	struct epoll_event event_setup_ = { .events = EPOLLIN }; // prepare the common part
};
