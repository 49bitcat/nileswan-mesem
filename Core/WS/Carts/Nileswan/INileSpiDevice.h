#pragma once

#include "pch.h"

class INileSpiDevice
{
public:
    virtual uint8_t SpiExchange(uint8_t tx) = 0;
	virtual ~INileSpiDevice() {}
};
