#pragma once
#include "WS/Carts/Nileswan/INileSpiDevice.h"
#include "WS/Carts/Nileswan/NileSpiBuffer.h"
#include "pch.h"
#include "WS/WsTypes.h"
#include "Utilities/ISerializable.h"

class WsCartNileswan;

class NileMCU final : public INileSpiDevice, public ISerializable
{
private:
    WsCartNileswan *parent;
    NileSpiBuffer txBuf, rxBuf;
    struct {
        bool boot_mode;
        bool boot_started;
        bool boot_waiting_ack;
        uint8_t boot_cmd;
        uint8_t boot_step;
        uint16_t boot_erase_count;
        uint32_t boot_dest_address;
        int16_t cdc_unget;
    } state;
    struct {
        uint8_t eeprom_mode;
        uint32_t save_id;
        uint16_t eeprom_data[1024];
    } persistent;

    void SendSpiResponse(uint16_t len, const void *buffer);
    void SendSpiBootAck(bool is_ack);
    uint8_t SpiExchangeBoot(uint8_t tx);
    void RtcTransfer(uint8_t cmd, uint8_t* buf);

public:
    NileMCU(WsCartNileswan *_parent);
	virtual ~NileMCU();

	void Reset(bool boot_mode);
	uint8_t SpiExchange(uint8_t tx) override;
	void Serialize(Serializer& s) override;
};
