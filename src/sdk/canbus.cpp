/******************************************************************
CanBus class for can-bus communication

Features:
- SocketCAN
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_imu/canbus.h"
#include <string.h>
#include <unistd.h>
#include <iostream>


CanBus::CanBus(const char* Name)
{
	strcpy(ifr_.ifr_name, Name);
}

CanBus::CanBus(const std::string& Name)
{
	strcpy(ifr_.ifr_name, Name.c_str());
}

bool CanBus::open()
{
	if ((fd_epoll_ = epoll_create(1)) < 0)
	{
		printf("failed to create epoll\n");
	}

	if ((if_obj_.socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
	{
		printf("cannot open socket for PF_CAN\n");
		return false;
	}
	else
	{
		event_setup_.data.ptr = &if_obj_; // remember the instance as private data
		if (epoll_ctl(fd_epoll_, EPOLL_CTL_ADD, if_obj_.socket_, &event_setup_))
		{
			printf("failed to add socket to epoll\n");
		}

		ioctl(if_obj_.socket_, SIOCGIFINDEX, &ifr_);

		memset(&addr_, 0, sizeof(addr_));
		addr_.can_family = AF_CAN;
		addr_.can_ifindex = ifr_.ifr_ifindex;

		// these settings are static and can be held out of the hot path
		iov_.iov_base = &frame_;
		msg_.msg_name = &addr_;
		msg_.msg_iov = &iov_;
		msg_.msg_iovlen = 1;
		msg_.msg_control = &ctrlmsg;

		if (bind(if_obj_.socket_, (struct sockaddr*)&addr_, sizeof(addr_)) < 0)
		{
			printf("cannot bind socket %d to %s\n", if_obj_.socket_, ifr_.ifr_name);
			return false;
		}

		return true;
	}
}

bool CanBus::isOpen()
{
	return is_open_;
}

bool CanBus::open(const std::string& Name)
{
	strcpy(ifr_.ifr_name, Name.c_str());

	return (is_open_ = open());
}

void CanBus::close()
{
	if (if_obj_.socket_ >= 0)
	{
		if (::close(if_obj_.socket_) < 0)
		{
			printf("failed to close socket %d\n", if_obj_.socket_);
		}
	}

	if (fd_epoll_ >= 0)
	{
		::close(fd_epoll_);
	}

	is_open_ = false;
}

ssize_t CanBus::read(unsigned int& ID, unsigned char* Data)
{
	ssize_t len = -1;

	if (if_obj_.socket_ >= 0)
	{
		struct can_frame frame;
		len = ::read(if_obj_.socket_, &frame, sizeof(struct can_frame));
		if (len > 0)
		{
			ID = frame.can_id;
			len = frame.can_dlc;
			memcpy(Data, frame.data, len);

#ifdef DEBUG
			printf("read ID: 0x%03X [%d] ", ID, len);
			for (int i = 0; i < len; ++i)
			{
				printf("0x%02X ", frame.data[i]);
			}
			printf("\n");
#endif
		}
		else
		{
			printf("failed to read from %s\n", ifr_.ifr_name);
		}
	}

	return len;
}

bool CanBus::write(unsigned int ID, size_t Len, const unsigned char* Data)
{
	if (if_obj_.socket_ >= 0)
	{
		struct can_frame frame;
		frame.can_id = ID;
		if (isExtended(frame.can_id))
		{
			frame.can_id |= CAN_EFF_FLAG;
		}
		frame.can_dlc = static_cast<unsigned char>(Len);
		memcpy(frame.data, Data, frame.can_dlc);

		if (::write(if_obj_.socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
		{
			printf("failed to write to %s\n", ifr_.ifr_name);
			return false;
		}
		else
		{
#ifdef DEBUG
			printf("sent ID: 0x%03X [%d] ", frame.can_id, frame.can_dlc);
			for (int i = 0; i < frame.can_dlc; ++i)
			{
				printf("0x%02X ", Data[i]);
			}
			printf("\n");
#endif

			return true;
		}
	}
	else
	{
		return false;
	}
}

std::size_t CanBus::eventTriggered(unsigned int& ID, unsigned char* Data)
{
	std::size_t readCount = 0;

	if (epoll_wait(fd_epoll_, &event_pending_, 1, -1) > 0)
	{
		struct IfInfo* obj = (IfInfo*)event_pending_.data.ptr;
		// these settings may be modified by recvmsg()
		char ctrlmsg[CMSG_SPACE(sizeof(struct timeval) + 3 * sizeof(struct timespec) + sizeof(__u32))];
		iov_.iov_len = sizeof(frame_);
		msg_.msg_namelen = sizeof(addr_);
		msg_.msg_controllen = sizeof(ctrlmsg);
		msg_.msg_flags = 0;

		ssize_t nbytes = recvmsg(obj->socket_, &msg_, 0);
		if (nbytes > 0)
		{
			readCount = nbytes;

			if (frame_.can_id & CAN_ERR_FLAG)
			{
				ID = frame_.can_id & (CAN_ERR_MASK|CAN_ERR_FLAG);
			}
			else if (frame_.can_id & CAN_EFF_FLAG)
			{
				ID = frame_.can_id & CAN_EFF_MASK;
			}
			else
			{
				ID = frame_.can_id & CAN_SFF_MASK;
			}
			memcpy(Data, frame_.data, frame_.len);
		}
	}

	fflush(stdout);

	return readCount;
}

bool CanBus::isExtended(unsigned int ID) const
{
	return (ID & 0xffff0000) == 0 ? false : true;
}
