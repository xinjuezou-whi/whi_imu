/******************************************************************
UsbCan class for can-bus communication

Features:
- usbcan module
- xxx

Dependency:
- sudo ln <path>/libcontrolcan.so /usr/lib/libcontrolcan.so
- sudo pluma /etc/udev/rules.d/99-usbcan.rules
  copy and paste following contents to file and save:
  ACTION=="add",SUBSYSTEMS=="usb", ATTRS{idVendor}=="04d8", ATTRS{idProduct}=="0053", GROUP="users", MODE="0777"
  then reboot

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_imu/usbcan.h"
#include "whi_imu/controlcan.h"
#include "whi_imu/printfColor.h"

#include <yaml-cpp/yaml.h>

#include <string.h>
#include <cstdio>
#include <iostream>

const std::string UsbCan::version ="\nWHI USBCAN driver VERSION 00.09.1\n\
Copyright © 2021-2025 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n";

const std::map<uint16_t, UsbCan::Timings> UsbCan::BAUDRATE_LIST = {
	{ 125, { 0x03, 0x1c } },
	{ 250, { 0x01, 0x1c } },
	{ 500, { 0x00, 0x1c } },
	{ 1000, { 0x00, 0x14 } } };

UsbCan::UsbCan(uint8_t BusAddr, uint16_t Baudrate,
	bool IsRemote/* = false*/, bool IsExtended/* = false*/, uint16_t UsbDeviceIndex/* = 0*/)
	: canusb_device_index_(UsbDeviceIndex)
{
	if (openDevice())
	{
		VCI_INIT_CONFIG config;
		config.AccCode = 0;
		config.AccMask = 0xFFFFFFFF;
		config.Filter = 1; // accept all
		if (BAUDRATE_LIST.find(Baudrate) == BAUDRATE_LIST.end())
		{
			config.Timing0 = BAUDRATE_LIST.at(500).timing_0_;
			config.Timing1 = BAUDRATE_LIST.at(500).timing_1_;
		}
		else
		{
			config.Timing0 = BAUDRATE_LIST.at(Baudrate).timing_0_;
			config.Timing1 = BAUDRATE_LIST.at(Baudrate).timing_1_;
		}
		config.Mode = 0; // normal mode	

		if (VCI_InitCAN(VCI_USBCAN2, canusb_device_index_, BusAddr, &config) == 1 && VCI_StartCAN(VCI_USBCAN2, canusb_device_index_, BusAddr) == 1)
		{
			is_open_ = true;
			remote_map_[BusAddr] = IsRemote;
			extended_map_[BusAddr] = IsExtended;
			state_map_[BusAddr] = true;
		}
		else
		{
			printf((std::string(LIGHT_RED) + "[error] failed to init can%d" + CLEANUP + "\n").c_str(), BusAddr);
		}

		if (!is_open_)
		{
			closeDevice();

			printf("[info] close usbcan device due to failure of can initialization\n");
		}
	}
}

UsbCan::UsbCan(const std::string& Config, uint16_t UsbDeviceIndex/* = 0*/)
{
	if (openDevice() && loadConfig(Config))
	{
		for (const auto& it : baudrate_map_)
		{
			VCI_INIT_CONFIG config;
			config.AccCode = 0;
			config.AccMask = 0xFFFFFFFF;
			config.Filter = 1; // accept all
			if (BAUDRATE_LIST.find(it.second) == BAUDRATE_LIST.end())
			{
				config.Timing0 = BAUDRATE_LIST.at(500).timing_0_;
				config.Timing1 = BAUDRATE_LIST.at(500).timing_1_;
			}
			else
			{
				config.Timing0 = BAUDRATE_LIST.at(it.second).timing_0_;
				config.Timing1 = BAUDRATE_LIST.at(it.second).timing_1_;
			}
			config.Mode = 0; // normal mode	

			if (VCI_InitCAN(VCI_USBCAN2, canusb_device_index_, it.first, &config) == 1 && VCI_StartCAN(VCI_USBCAN2, canusb_device_index_, it.first) == 1)
			{
				is_open_ |= true;
				state_map_[it.first] = true;
			}
			else
			{
				printf((std::string(LIGHT_RED) + "[error] failed to init can%d" + CLEANUP + "\n").c_str(), it.first);
			}
		}

		if (!is_open_)
		{
			closeDevice();

			printf("close usbcan device due to failure of can initialization\n");
		}
	}
}

UsbCan::~UsbCan()
{
	close();
}

bool UsbCan::isOpen()
{
	return is_open_;
}

void UsbCan::close()
{
	if (is_open_)
	{
		for (const auto& it : state_map_)
		{
			if (it.second)
			{
				VCI_ResetCAN(VCI_USBCAN2, canusb_device_index_, it.first);
			}
		}

		closeDevice();

		is_open_ = false;
	}	
}

void UsbCan::clearBuffer(uint8_t BusAddr)
{
	if (isBusOpen(BusAddr))
	{
		VCI_ClearBuffer(VCI_USBCAN2, canusb_device_index_, BusAddr);
	}
}

bool UsbCan::write(uint8_t BusAddr, unsigned int ID, size_t Len, const uint8_t* Data)
{
	if (!is_open_ || !isBusOpen(BusAddr))
	{
		return false;
	}

	VCI_CAN_OBJ send[1];
	send[0].ID = ID;
	send[0].SendType = 0;
	send[0].RemoteFlag = getAtribMapValue(remote_map_, BusAddr); // 0 statds for data
	send[0].ExternFlag = getAtribMapValue(extended_map_, BusAddr); // 0 stands for stardard frame
	send[0].DataLen = 8;

	memset(send[0].Data, 0, sizeof(send[0].Data));
	for (size_t i = 0; i < Len; ++i)
	{
		send[0].Data[i] = Data[i];
	}
#ifdef DEBUG
	std::cout << "write " << send[0].ID << " " << int(send[0].ID & 0x000000ff) << " " << int((send[0].ID & 0x0000ff00) >> 8) << " " <<
		int(send[0].Data[0]) << " " << int(send[0].Data[1]) << " " << int(send[0].Data[2]) << " " << int(send[0].Data[3]) << " " <<
		int(send[0].Data[4]) << " " << int(send[0].Data[5]) << " " << int(send[0].Data[6]) << " " << int(send[0].Data[7]) << std::endl;
#endif

	unsigned int len = 0;
	const std::lock_guard<std::mutex> lock(mtx_[BusAddr]);
	{
		len = VCI_Transmit(VCI_USBCAN2, canusb_device_index_, BusAddr, send, 1);
	}

	return len;
}

ssize_t UsbCan::read(uint8_t BusAddr, std::vector<DataPack>& Pack)
{
	if (!is_open_ || !isBusOpen(BusAddr))
	{
		return 0;
	}

	const size_t BUF_LEN = 3000;
	VCI_CAN_OBJ rec[BUF_LEN];

	unsigned int len = 0;
	const std::lock_guard<std::mutex> lock(mtx_[BusAddr]);
	{
		len = VCI_Receive(VCI_USBCAN2, canusb_device_index_, BusAddr, rec, BUF_LEN, 10);
	}

	if (len < 0)
	{
		len = 0;
	}
	else if (len > 100)
	{
		len = 100;
	}
	Pack.resize(len);

	for (size_t i = 0; i < len; ++i)
	{
		Pack[i].id_ = (uint32_t)(rec[i].ID);
		Pack[i].length_ = rec[i].DataLen;
		memcpy(Pack[i].data_.data(), rec[i].Data, rec[i].DataLen);
	}

	return len;
}

ssize_t UsbCan::read(std::vector<DataPack>& Pack)
{
	unsigned int totalLen = 0;
	for (const auto& it : state_map_)
	{
		if (it.second)
		{
			totalLen += read(it.first, Pack);
		}
	}

	return totalLen;
}

size_t UsbCan::increaseReference()
{
	return ++reference_count_;
}

size_t UsbCan::decreaseReference()
{
	return --reference_count_;
}

bool UsbCan::openDevice()
{
	std::cout << version << std::endl;

	VCI_BOARD_INFO devInfoList[50];
	auto foundNum = VCI_FindUsbDevice2(devInfoList);
	if (foundNum > 0)
	{
		if (VCI_OpenDevice(VCI_USBCAN2, canusb_device_index_, 0) == 1)
		{
			return true;
		}
		else
		{
			printf((std::string(LIGHT_RED) + "[error] failed to open usbcan device %d" + CLEANUP + "\n").c_str(),
				canusb_device_index_);
		}
	}
	else
	{
		printf((std::string(LIGHT_RED) + "[error] cannot find usbcan device" + CLEANUP + "\n").c_str());
	}

	return false;
}

void UsbCan::resetDevice()
{
	VCI_UsbDeviceReset(VCI_USBCAN2, canusb_device_index_, 0/*reserved*/);
}

void UsbCan::closeDevice()
{
	VCI_CloseDevice(VCI_USBCAN2, canusb_device_index_);
}

bool UsbCan::loadConfig(const std::string& Config)
{
    std::map<int, int> attribMap;

    try
    {
        YAML::Node node = YAML::LoadFile(Config);

        const auto& baudrate = node["baudrate"];
        for (const auto& it : baudrate)
        {
            baudrate_map_.emplace(it.first.as<int>(), it.second.as<int>());
        }

		const auto& remote = node["is_remote"];
		if (remote)
		{
			for (const auto& it : remote)
			{
				remote_map_.emplace(it.first.as<int>(), it.second.as<bool>());
			}
		}
		const auto& extended = node["is_extended"];
		if (extended)
		{
			for (const auto& it : extended)
			{
				extended_map_.emplace(it.first.as<int>(), it.second.as<bool>());
			}
		}

		return true;
    }
    catch (const std::exception& e)
    {
        std::cout << "failed to load usbcan attributes file " << Config << " with error: " << e.what() << std::endl;

		return false;
    }
}

bool UsbCan::isBusOpen(uint8_t BusAddr)
{
	if (auto found = state_map_.find(BusAddr); found != state_map_.end())
	{
		return found->second;
	}
	else
	{
		return false;
	}
}

uint8_t UsbCan::getAtribMapValue(const std::map<int, bool>& Map, int Key)
{
	if (auto found = Map.find(Key); found != Map.end())
	{
		return found->second ? 1 : 0;
	}
	else
	{
		return 0;
	}
}
