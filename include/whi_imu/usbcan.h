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

Changelog:
2021-07-24: Initial version
2021-xx-xx: xxx
******************************************************************/
#pragma once
#include <cstdint>
#include <sys/types.h>
#include <vector>
#include <array>
#include <map>
#include <mutex>

class UsbCan
{
public:
	enum BaudRate { Kbps_125 = 0, Kbps_250, Kbps_500, Kbps_1000, KbpsSum };
	enum BusAddr { CAN0 = 0x01, CAN1 = 0x10, BOTH = 0x11 };
	struct DataPack
	{
		DataPack() {};
		DataPack(uint32_t ID, std::array<uint8_t, 8> Data, size_t Len)
			: id_(ID), data_(Data), length_(Len) {};
		uint32_t id_{ 0 };
		std::array<uint8_t, 8> data_;
		size_t length_{ 0 };
	};

protected:
	struct Timings
	{
		// default as 500Kbps
		uint8_t timing_0_{ 0x00 };
		uint8_t timing_1_{ 0x1c };
	};

public:
	UsbCan() = delete;
	UsbCan(uint8_t BusAddr, uint16_t Baudrate,
		bool IsRemote = false, bool IsExtended = false, uint16_t UsbDeviceIndex = 0);
	UsbCan(const std::string& Config, uint16_t UsbDeviceIndex = 0);
	~UsbCan();

public:
	bool isOpen();
	void close();
	void clearBuffer(uint8_t BusAddr);
	bool write(uint8_t BusAddr, unsigned int ID, size_t Len, const uint8_t* Data);
	ssize_t read(uint8_t BusAddr, std::vector<DataPack>& Pack);
	ssize_t read(std::vector<DataPack>& Pack);
	size_t increaseReference();
	size_t decreaseReference();

protected:
	bool openDevice();
	void resetDevice();
	void closeDevice();
	bool loadConfig(const std::string& Config);
	bool isBusOpen(uint8_t BusAddr);

protected:
	static uint8_t getAtribMapValue(const std::map<int, bool>& Map, int Key);

protected:
	uint16_t canusb_device_index_{ 0 };
	bool is_open_{ false };
	size_t reference_count_{ 0 };
	std::mutex mtx_[2];
	std::map<int, int> baudrate_map_;
	std::map<int, bool> remote_map_;
	std::map<int, bool> extended_map_;
	std::map<int, bool> state_map_;

protected:
	static const std::string version;
	static const std::map<uint16_t, Timings> BAUDRATE_LIST;
};
