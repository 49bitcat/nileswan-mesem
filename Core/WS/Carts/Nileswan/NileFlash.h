#pragma once
#include "WS/Carts/Nileswan/INileSpiDevice.h"
#include "WS/Carts/Nileswan/NileSpiBuffer.h"
#include "pch.h"
#include "WS/WsTypes.h"
#include "Utilities/ISerializable.h"

class NileFlash final : public INileSpiDevice, public ISerializable
{
private:
    NileSpiBuffer txBuf, rxBuf;
    uint8_t sr1, sr2, sr3;
    uint8_t mode;
    uint32_t position;
    bool sleeping;

public:
    FILE *file = nullptr;

    NileFlash();
	virtual ~NileFlash();

	void Reset();
	uint8_t SpiExchange(uint8_t tx) override;
	void Serialize(Serializer& s) override;
};
