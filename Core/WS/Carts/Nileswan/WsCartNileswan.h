#pragma once
#include "WS/Carts/Nileswan/INileSpiDevice.h"
#include "WS/Carts/Nileswan/NileFlash.h"
#include "WS/Carts/Nileswan/NileMCU.h"
#include "WS/Carts/Nileswan/NileTF.h"
#include "pch.h"
#include "WS/WsTypes.h"
#include "WS/Carts/WsCart.h"
#include "Utilities/ISerializable.h"
#include "Shared/MemoryType.h"

enum WwState
{
	WW_STATE_READ = 0,
	WW_STATE_UNLOCK_1,
	WW_STATE_UNLOCK_2,
	WW_STATE_FAST,
	WW_STATE_FAST_WRITE,
	WW_STATE_WRITE,
	WW_STATE_ERASE
};

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
    bool flash_enable;
    uint16_t bank_rom0, bank_rom1, bank_romL, bank_ram;
    uint8_t nile_pow_cnt, nile_emu_cnt;
    uint16_t nile_spi_cnt, nile_bank_mask;
    int8_t nile_fpga_core;
    WwState nile_ww_state;

    int sram_banks, psram_banks;

    uint8_t *buffer_psram = nullptr;
    uint8_t *buffer_sram = nullptr;
    uint8_t buffer_ipc[NILE_IPC_SIZE];
    uint8_t buffer_spi_tx[2][NILE_SPI_SIZE];
    uint8_t buffer_spi_rx[2][NILE_SPI_SIZE];

    void OnSpiCntUpdate(uint16_t previous);
    void OnPowCntUpdate(uint8_t previous);

protected:
    uint8_t SpiExchange(uint8_t tx);
    void ResolveBank(uint32_t address, uint8_t** buffer, bool write, bool is_debugger);
	int GetSpiBankIndex(bool is_swan);

public:
    NileFlash flash;
    NileMCU mcu;
    NileTF tf;

	WsCartNileswan(int _psram_banks, int _sram_banks);
	virtual ~WsCartNileswan();

	void Reset();
	void FpgaReset();
	bool IsTFPowered();
	void RefreshMappings() override;

	uint8_t ReadPort(uint16_t port) override;
	void WritePort(uint16_t port, uint8_t value) override;

	uint8_t ReadMemory(uint32_t addr) override;
	void WriteMemory(uint32_t addr, uint8_t value) override;

	void Serialize(Serializer& s) override;
};
