#pragma once
#include "pch.h"
#include "WS/WsTypes.h"
#include "WS/Carts/WsCart.h"
#include "WS/Carts/WsRtc.h"
#include "WS/Carts/Nileswan/INileSpiDevice.h"
#include "WS/Carts/Nileswan/NileFlash.h"
#include "WS/Carts/Nileswan/NileMCU.h"
#include "WS/Carts/Nileswan/NileTF.h"
#include "Utilities/ISerializable.h"
#include "Shared/MemoryType.h"
#include "Debugger/AddressInfo.h"

#define NILE_IPC_SIZE 512
#define NILE_SPI_SIZE 512

#define NILE_EMULATED_BOARD_REVISION 0x02
#define NILE_EMULATED_MCU_MAJOR 1
#define NILE_EMULATED_MCU_MINOR 2

#define NILE_MCU_MAX_PER_USB_CDC_PACKET 128

#define NILE_TF_STOP_TRANSFER_BUSY_DELAY_BYTES 8
#define NILE_TF_DATA_BLOCK_READ_DELAY_BYTES 16

class WsCartNileswan final : public WsCart
{
private:
	enum class WwState : uint8_t
	{
		READ = 0,
		UNLOCK_1,
		UNLOCK_2,
		FAST,
		FAST_WRITE,
		WRITE,
		ERASE
	};

	WsNileCartState _nstate;
	WwState wwState;

	uint8_t buffer_spi_tx[2][NILE_SPI_SIZE];
	uint8_t buffer_spi_rx[2][NILE_SPI_SIZE];

	void OnSpiCntUpdate(uint16_t previous);
	void OnPowCntUpdate(uint8_t previous);

protected:
	 WsRtc* _rtc = nullptr;

	uint8_t SpiExchange(uint8_t tx);
	void ResolveBank(uint32_t address, uint8_t** buffer, bool write, bool is_debugger);

	int GetSpiBankIndex(bool is_swan);
	MemoryType GetMemoryTypeForBank(uint16_t bank, bool rom, bool linear);

public:
	uint8_t* buffer_psram = nullptr;
	uint8_t* buffer_sram = nullptr;
	uint8_t buffer_ipc[NILE_IPC_SIZE];
	int psram_banks, sram_banks;

	NileFlash flash;
	NileMCU mcu;
	NileTF tf;

	WsCartNileswan(WsRtc *rtc, int _psram_banks, int _sram_banks);
	virtual ~WsCartNileswan();

	WsNileCartState& GetNileState() { return _nstate; }

	void Reset();
	void FpgaReset();
	void LoadBattery() override;
	void SaveBattery() override;
	void RefreshMappings() override;

	bool IsTFPowered();

	uint32_t GetSelectedBank(uint8_t index);
	AddressInfo GetAbsoluteAddress(uint32_t relAddr);

	uint8_t ReadPort(uint16_t port) override;
	void WritePort(uint16_t port, uint8_t value) override;

	bool InternalReadCart(uint32_t addr, uint8_t& value) override;
	bool InternalWriteCart(uint32_t addr, uint8_t value) override;

	void Serialize(Serializer& s) override;
	
	WsRtc* GetRtc() { return _rtc; }
};
