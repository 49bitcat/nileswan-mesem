#pragma once

#include "pch.h"

class IWsSerialDevice
{
public:
	//Returns true if the device pushes bytes to the console by itself.
	virtual bool IsSerialAutoPush() = 0;
	virtual bool CanReceiveSerial() = 0;
	virtual bool CanSendSerial() = 0;
	virtual void ReceiveSerial(uint8_t value) = 0;
	virtual uint8_t SendSerial() = 0;

	virtual ~IWsSerialDevice() {}
};