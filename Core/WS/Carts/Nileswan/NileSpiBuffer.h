#pragma once
#include "pch.h"
#include "WS/WsTypes.h"
#include "Utilities/ISerializable.h"

class NileSpiBuffer final : public ISerializable
{
public:
    uint8_t *data;
    size_t pos, size;

	NileSpiBuffer(size_t size);
	virtual ~NileSpiBuffer();

	void Reset();
	void Push(const uint8_t *data, size_t length);
	bool Pop(uint8_t *data, size_t length);

	void Serialize(Serializer& s) override;
};
