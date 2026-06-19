#pragma once
#include "WS/Carts/Nileswan/INileSpiDevice.h"
#include "WS/Carts/Nileswan/NileSpiBuffer.h"
#include "pch.h"
#include "WS/WsTypes.h"
#include "Utilities/ISerializable.h"

class WsCartNileswan;

class NileTF final : public INileSpiDevice, public ISerializable
{
private:
    WsCartNileswan *parent;
    NileSpiBuffer txBuf, rxBuf;
    uint8_t status;
    bool is_acmd;
    bool reading;
    uint8_t writing;

public:
    FILE *file = nullptr;

    NileTF(WsCartNileswan *_parent);
	virtual ~NileTF();

	void Reset();
	uint8_t SpiExchange(uint8_t tx) override;
	void Serialize(Serializer& s) override;
};
