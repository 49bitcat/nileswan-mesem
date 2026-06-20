#include "pch.h"
#include "Shared/MemoryType.h"
#include "Shared/MessageManager.h"
#include "Utilities/Serializer.h"
#include "WS/Carts/WsCart.h"
#include "WS/Carts/WsRtc.h"
#include "WS/Carts/Nileswan/WsCartNileswan.h"
#include "WS/WsMemoryManager.h"
#include "hardware.h"

WsCartNileswan::WsCartNileswan(WsRtc *rtc, int _psram_banks, int _sram_banks) : WsCart(),
    psram_banks(_psram_banks), sram_banks(_sram_banks),
    flash(NileFlash()), mcu(NileMCU(this)), tf(NileTF(this))
{
	 _rtc = rtc;
	_hasCustomReadHandler = true;
	_hasCustomWriteHandler = true;
	
    buffer_psram = new uint8_t[psram_banks * 0x10000];
    buffer_sram = new uint8_t[sram_banks * 0x10000];

	Reset();
}

WsCartNileswan::~WsCartNileswan()
{
	delete[] buffer_psram;
	delete[] buffer_sram;
}

void WsCartNileswan::Reset()
{
	_nstate.FpgaCore = -1;
	FpgaReset();

	flash.Reset();
	mcu.Reset(false);
	tf.Reset();
}

void WsCartNileswan::FpgaReset()
{
	_state.SelectedBanks[0] = 0xFF;
	_state.SelectedBanks[1] = 0xFF;
	_state.SelectedBanks[2] = 0xFF;
	_state.SelectedBanks[3] = 0xFF;
	_state.ExtSelectedBanks[0] = 0x00;
	_state.ExtSelectedBanks[1] = 0xFF;
	_state.ExtSelectedBanks[2] = 0xFF;
	_state.ExtSelectedBanks[3] = 0xFF;
	_state.RomInRamBank = false;
	_nstate.SpiCnt = 0;
	_nstate.PowCnt = NILE_POW_UNLOCK;
	_nstate.BankMask = 0xFFFF;
	_nstate.EmuCnt = 0;
	wwState = WwState::READ;

	memset(buffer_ipc, 0, sizeof(buffer_ipc));

	if(_rtc) {
		_rtc->Reset();
	}
}

void WsCartNileswan::LoadBattery()
{
	WsCart::LoadBattery();
	_rtc->LoadBattery();
}

void WsCartNileswan::SaveBattery()
{
	WsCart::SaveBattery();
	_rtc->SaveBattery();
}

bool WsCartNileswan::IsTFPowered() const
{
	return _nstate.PowCnt & NILE_POW_TF;
}

uint32_t WsCartNileswan::GetMaskedPsramSize() const
{
	return ((_nstate.BankMask & (psram_banks - 1)) + 1) << 16;
}

uint32_t WsCartNileswan::GetMaskedSramSize() const
{
	return (((_nstate.BankMask >> 12) & (sram_banks - 1)) + 1) << 16;
}

// I/O handling

void WsCartNileswan::OnSpiCntUpdate(uint16_t prev_spi_cnt)
{
	if(!(_nstate.SpiCnt & NILE_SPI_390KHZ) && !(_nstate.PowCnt & NILE_POW_CLOCK)) {
		return;
	}

	if((_nstate.SpiCnt & NILE_SPI_DEV_MASK) != NILE_SPI_DEV_FLASH) {
		flash.Reset();
	}

	if(_nstate.SpiCnt & NILE_SPI_BUSY) {
		const char* deviceName = "none";
		if((_nstate.SpiCnt & NILE_SPI_DEV_MASK) == NILE_SPI_DEV_TF) {
			deviceName = "TF card";
		} else if((_nstate.SpiCnt & NILE_SPI_DEV_MASK) == NILE_SPI_DEV_FLASH) {
			deviceName = "SPI flash";
		} else if((_nstate.SpiCnt & NILE_SPI_DEV_MASK) == NILE_SPI_DEV_MCU) {
			deviceName = "MCU";
		}

		uint8_t* txBuffer = buffer_spi_tx[GetSpiBankIndex(false)];
		uint8_t* rxBuffer = buffer_spi_rx[GetSpiBankIndex(false)];
		uint32_t length = (_nstate.SpiCnt & 0x1FF) + 1;
		uint32_t pos = 0;
		uint16_t mode = _nstate.SpiCnt & NILE_SPI_MODE_MASK;
		if(mode == NILE_SPI_MODE_WAIT_READ) {
			int32_t timeout = 8192;
			uint32_t bytes_skipped = 0;
			while(--timeout) {
				if((rxBuffer[0] = SpiExchange(0xFF)) != 0xFF) {
					break;
				}
				bytes_skipped++;
			}
			if(timeout <= 0) {
				MessageManager::Log("[Nile] SPI wait read timeout for " + std::string(deviceName));
				printf("nileswan/spi: !!! WAIT_READ timeout for %s !!!\n", deviceName);
				return;
			} else if(bytes_skipped > 0) {
				printf("nileswan/spi: skipped %d bytes\n", bytes_skipped);
			}
			pos++;
		}
		bool modeReads = mode != NILE_SPI_MODE_WRITE;
		bool modeWrites = mode == NILE_SPI_MODE_WRITE || mode == NILE_SPI_MODE_EXCH;
		for(; pos < length; pos++) {
			uint8_t rx = SpiExchange(modeWrites ? txBuffer[pos] : 0xFF);
			if(modeReads) {
				rxBuffer[pos] = rx;
			}
		}
		printf("nileswan/spi: %s %d bytes %s %s",
			modeReads ? (modeWrites ? "exchanging" : "reading") : (modeWrites ? "writing" : "???"),
			length,
			modeReads ? (modeWrites ? "with" : "from") : (modeWrites ? "to" : "with"),
			deviceName);
		if(modeWrites) {
			printf(" [%02x", txBuffer[0]);
			for(pos = 1; pos < length; pos++) {
				printf(" %02x", txBuffer[pos]);
			}
			printf("]");
		}
		if(modeReads) {
			printf(" [%02x", rxBuffer[0]);
			for(pos = 1; pos < length; pos++) {
				printf(" %02x", rxBuffer[pos]);
			}
			printf("]");
		}
		printf("\n");
		_nstate.SpiCnt = _nstate.SpiCnt & ~NILE_SPI_BUSY;
	}
	fflush(stdout);
}

void WsCartNileswan::OnPowCntUpdate(uint8_t newValue)
{
	uint8_t oldValue = _nstate.PowCnt;
	_nstate.PowCnt = newValue;
	if(!(_nstate.PowCnt & NILE_POW_TF)) {
		tf.Reset();
	}
	if(!(oldValue & NILE_POW_MCU_RESET) && (newValue & NILE_POW_MCU_RESET)) {
		bool bootloader_mode = (newValue & NILE_POW_MCU_BOOT0) != 0;
		printf("nileswan/mcu: reset, in %s mode\n", bootloader_mode ? "bootloader" : "native");
		mcu.Reset(bootloader_mode);
	}
}

uint8_t WsCartNileswan::ReadPort(uint16_t index)
{
	switch(index) {
		case IO_BANK_ROM_LINEAR:
			return _state.SelectedBanks[0];
		case IO_BANK_RAM:
			return _state.SelectedBanks[1];
		case IO_BANK_ROM0:
			return _state.SelectedBanks[2];
		case IO_BANK_ROM1:
			return _state.SelectedBanks[3];
	}

	if(_nstate.PowCnt & NILE_POW_IO_2003) {
		switch(index) {
			case 0xCA:
			case 0xCB:
				return _rtc->ReadPort(index);
			case IO_CART_FLASH:
				return _state.RomInRamBank ? 1 : 0;
			case IO_BANK_2003_ROM_LINEAR:
				return _state.SelectedBanks[0];
			case IO_BANK_2003_RAM:
				return _state.SelectedBanks[1];
			case IO_BANK_2003_RAM + 1:
				return _state.ExtSelectedBanks[1];
			case IO_BANK_2003_ROM0:
				return _state.SelectedBanks[2];
			case IO_BANK_2003_ROM0 + 1:
				return _state.ExtSelectedBanks[2];
			case IO_BANK_2003_ROM1:
				return _state.SelectedBanks[3];
			case IO_BANK_2003_ROM1 + 1:
				return _state.ExtSelectedBanks[3];
		}
	}

	if(_nstate.PowCnt & NILE_POW_IO_NILE) {
		switch(index) {
			case IO_NILE_POW_CNT:
				return _nstate.PowCnt;
			case IO_NILE_SEG_MASK:
				return _nstate.BankMask;
			case IO_NILE_SEG_MASK + 1:
				return _nstate.BankMask >> 8;
			case IO_NILE_SPI_CNT:
				return _nstate.SpiCnt;
			case IO_NILE_SPI_CNT + 1:
				return _nstate.SpiCnt >> 8;
			case IO_NILE_IRQ_STATUS:
				return _nstate.IrqStatus;
			case IO_NILE_IRQ_ENABLE:
				return _nstate.IrqEnable;
			case IO_NILE_BOARD_REVISION:
				return NILE_EMULATED_BOARD_REVISION;
		}
	}

	// TODO what is the open bus value?
	// return 0xFF;
	return 0x00;
}

void WsCartNileswan::WritePort(uint16_t index, uint8_t value)
{
	switch(index) {
		case IO_BANK_ROM_LINEAR:
			_state.SelectedBanks[0] = value;
			RefreshMappingsRange(4, 15);
			break;
		case IO_BANK_RAM:
			_state.SelectedBanks[1] = value;
			RefreshMappingsRange(1, 1);
			break;
		case IO_BANK_ROM0:
			_state.SelectedBanks[2] = value;
			RefreshMappingsRange(2, 2);
			break;
		case IO_BANK_ROM1:
			_state.SelectedBanks[3] = value;
			RefreshMappingsRange(3, 3);
			break;
		case IO_NILE_POW_CNT:
			if(!(_nstate.PowCnt & NILE_POW_IO_NILE) && value != NILE_POW_UNLOCK) {
				break;
			}
			bool refreshMappingsRequired = ((_nstate.PowCnt ^ value) & NILE_POW_SRAM) != 0;
			OnPowCntUpdate(value);
			if(refreshMappingsRequired) {
			    RefreshMappingsRange(1, 1);
			}
			break;
	}

	if(_nstate.PowCnt & NILE_POW_IO_2003) {
		switch(index) {
			case 0xCA:
			case 0xCB:
				_rtc->WritePort(index, value);
				break;
			case IO_CART_FLASH:
				_state.RomInRamBank = value & 0x01;
				RefreshMappingsRange(1, 1);
				break;
			case IO_BANK_2003_ROM_LINEAR:
				_state.SelectedBanks[0] = value;
				RefreshMappingsRange(4, 15);
				break;
			case IO_BANK_2003_RAM:
				_state.SelectedBanks[1] = value;
				RefreshMappingsRange(1, 1);
				break;
			case IO_BANK_2003_RAM + 1:
				_state.ExtSelectedBanks[1] = value;
				RefreshMappingsRange(1, 1);
				break;
			case IO_BANK_2003_ROM0:
				_state.SelectedBanks[2] = value;
				RefreshMappingsRange(2, 2);
				break;
			case IO_BANK_2003_ROM0 + 1:
				_state.ExtSelectedBanks[2] = value;
				RefreshMappingsRange(2, 2);
				break;
			case IO_BANK_2003_ROM1:
				_state.SelectedBanks[3] = value;
				RefreshMappingsRange(3, 3);
				break;
			case IO_BANK_2003_ROM1 + 1:
				_state.ExtSelectedBanks[3] = value;
				RefreshMappingsRange(3, 3);
				break;
		}
	}

	if(_nstate.PowCnt & NILE_POW_IO_NILE) {
		switch(index) {
			case IO_NILE_WARMBOOT_CNT:
				_nstate.FpgaCore = value & 0x3;
				printf("nileswan/fpga: warmboot to core %d\n", _nstate.FpgaCore);
				FpgaReset();
				RefreshMappings();
				break;
			case IO_NILE_SEG_MASK:
				_nstate.BankMask = (_nstate.BankMask & 0xFF00) | value;
				RefreshMappings();
				break;
			case IO_NILE_SEG_MASK + 1:
				_nstate.BankMask = (_nstate.BankMask & 0xFF) | (value << 8);
				RefreshMappings();
				break;
			case IO_NILE_IRQ_ENABLE:
				_nstate.IrqEnable = value & 0x03;
				break;
			case IO_NILE_SPI_CNT: {
				uint16_t newSpiCnt = (_nstate.SpiCnt & 0xFF00) | value;
				if(_nstate.SpiCnt & NILE_SPI_BUSY) {
					printf("nileswan/spi: BUG trying to write to SPI control while transfer active (control %04X => %04X)\n",
						_nstate.SpiCnt,
						newSpiCnt);
					break;
				}
				_nstate.SpiCnt = newSpiCnt;
			} break;
			case IO_NILE_SPI_CNT + 1: {
				uint16_t newSpiCnt = (_nstate.SpiCnt & 0xFF) | (value << 8);
				if(_nstate.SpiCnt != newSpiCnt && (newSpiCnt | NILE_SPI_BUSY) == _nstate.SpiCnt) {
					printf("nileswan/spi: abort\n");
				} else if(_nstate.SpiCnt & NILE_SPI_BUSY) {
					printf("nileswan/spi: BUG trying to write to SPI control while transfer active (control %04X => %04X)\n",
						_nstate.SpiCnt,
						newSpiCnt);
					break;
				}
				uint16_t oldSpiCnt = _nstate.SpiCnt;
				_nstate.SpiCnt = newSpiCnt;
				printf("nileswan/spi: control = %04X\n", _nstate.SpiCnt);
				OnSpiCntUpdate(oldSpiCnt);
				if((oldSpiCnt ^ newSpiCnt) & NILE_SPI_BUFFER_IDX) {
					RefreshMappings();
				}
			} break;
			case IO_NILE_EMU_CNT:
				_nstate.EmuCnt = value & 0x3F;
				RefreshMappings();
				break;
		}
	}
}

// SPI routing

int WsCartNileswan::GetSpiBankIndex(bool consoleSide) const
{
	bool backBuffer = (_nstate.SpiCnt & NILE_SPI_BUFFER_IDX) != 0;
	backBuffer ^= consoleSide;
	return backBuffer ? 0 : 1;
}

uint8_t WsCartNileswan::SpiExchange(uint8_t tx)
{
	if((_nstate.SpiCnt & NILE_SPI_DEV_MASK) == NILE_SPI_DEV_TF) {
		return tf.SpiExchange(tx);
	} else if((_nstate.SpiCnt & NILE_SPI_DEV_MASK) == NILE_SPI_DEV_FLASH) {
		return flash.SpiExchange(tx);
	} else if((_nstate.SpiCnt & NILE_SPI_DEV_MASK) == NILE_SPI_DEV_MCU) {
		return mcu.SpiExchange(tx);
	} else {
		return 0xFF;
	}
}

// Memory routing

uint32_t WsCartNileswan::GetSelectedBank(uint8_t index) const
{
	if(index == 0) {
		return _state.SelectedBanks[0];
	} else {
		return (_state.ExtSelectedBanks[index - 1] << 8) | _state.SelectedBanks[index];
	}
}

AddressInfo WsCartNileswan::GetAbsoluteAddress(uint32_t relAddr)
{
	uint8_t* ptr = ResolveBank(relAddr, false, true);

	if(ptr >= buffer_psram && ptr < buffer_psram + (psram_banks * 0x10000)) {
		return { (int)(ptr - buffer_psram), MemoryType::WsPrgRom };
	} else if(ptr >= buffer_sram && ptr < buffer_sram + (sram_banks * 0x10000)) {
		return { (int)(ptr - buffer_sram), MemoryType::WsCartRam };
	} else if(ptr >= _prgRom && ptr < _prgRom + _prgRomSize) {
		return { (int)(ptr - _prgRom), MemoryType::WsNileBootrom };
	} else if(ptr >= buffer_ipc && ptr < buffer_ipc + NILE_IPC_SIZE) {
		return { (int)(ptr - buffer_ipc), MemoryType::WsNileIpc };
	} else if(ptr >= buffer_spi_tx[0] && ptr < buffer_spi_tx[0] + NILE_SPI_SIZE) {
		return { (int)(ptr - buffer_spi_tx[0]), MemoryType::WsNileSpiTx };
	} else if(ptr >= buffer_spi_tx[1] && ptr < buffer_spi_tx[1] + NILE_SPI_SIZE) {
		return { (int)(ptr - buffer_spi_tx[1]), MemoryType::WsNileSpiTx };
	} else if(ptr >= buffer_spi_rx[0] && ptr < buffer_spi_rx[0] + NILE_SPI_SIZE) {
		return { (int)(ptr - buffer_spi_rx[0]), MemoryType::WsNileSpiRx };
	} else if(ptr >= buffer_spi_rx[1] && ptr < buffer_spi_rx[1] + NILE_SPI_SIZE) {
		return { (int)(ptr - buffer_spi_rx[1]), MemoryType::WsNileSpiRx };
	}

	return { -1, MemoryType::None };
}

void WsCartNileswan::RefreshMappingsRange(int start, int end)
{
	bool readsNeedCode = (_nstate.EmuCnt & (NILE_EMU_FLASH_FSM | NILE_EMU_LODSW_TRICK)) != 0;
	bool writesNeedCode = (_nstate.EmuCnt & NILE_EMU_FLASH_FSM) != 0;
	for(int i = start; i <= end; i++) {
		AddressInfo info = GetAbsoluteAddress(i << 16);
		Map(i << 16, (i << 16) + 0xFFFF, info.Type, info.Address, i > 1);
	}

	_hasCustomReadHandler = readsNeedCode;
	_hasCustomWriteHandler = writesNeedCode;
}

void WsCartNileswan::RefreshMappings()
{
	if(_emu == NULL || _memoryManager == NULL) {
		return;
	}

	_emu->RegisterMemory(MemoryType::WsNileSpiTx, buffer_spi_tx[GetSpiBankIndex(true)], NILE_SPI_SIZE);
	_emu->RegisterMemory(MemoryType::WsNileSpiRx, buffer_spi_rx[GetSpiBankIndex(true)], NILE_SPI_SIZE);

	RefreshMappingsRange(1, 15);
}

uint16_t WsCartNileswan::ResolveBankValue(int cpuBank) const
{
	bool toSram = (cpuBank == 1) && !_state.RomInRamBank;
	uint16_t maskOff = toSram ? 0xF : 0x1FF;
	uint16_t maskOn = toSram ? (_nstate.BankMask >> 12) : (_nstate.BankMask & 0x1FF);

	if(cpuBank == 1) {
		return GetSelectedBank(1) & ((_nstate.BankMask & NILE_SEG_SRAM_LOCK) ? maskOn : maskOff);
	} else if(cpuBank == 2) {
		return GetSelectedBank(2) & ((_nstate.BankMask & NILE_SEG_ROM0_LOCK) ? maskOn : maskOff);
	} else if(cpuBank == 3) {
		return GetSelectedBank(3) & ((_nstate.BankMask & NILE_SEG_ROM1_LOCK) ? maskOn : maskOff);
	} else {
		return (((_state.SelectedBanks[0] & 0x1F) << 4) | cpuBank) & maskOn;
	}
}

uint8_t* WsCartNileswan::ResolveBank(uint32_t address, bool write, bool isDebugger)
{
	uint8_t cpuBank = (address >> 16) & 0xF;
	bool toSram = (cpuBank == 1) && !_state.RomInRamBank;

	uint32_t physBank = ResolveBankValue(cpuBank);
	uint32_t physAddr = (physBank << 16) | (address & 0xFFFF);

	if(toSram) {
		if(physBank < sram_banks) {
			if(_nstate.PowCnt & NILE_POW_SRAM) {
				if(_nstate.EmuCnt & NILE_EMU_SRAM_32KB) {
					physAddr &= ~0x8000;
				}
				return buffer_sram + physAddr;
			}
		} else if(physBank == NILE_SEG_RAM_IPC) {
			return buffer_ipc + (physAddr & (NILE_IPC_SIZE - 1));
		} else if(physBank == NILE_SEG_RAM_TX && (write || isDebugger)) {
			return buffer_spi_tx[GetSpiBankIndex(true)] + (physAddr & (NILE_SPI_SIZE - 1));
		}
	} else {
		if(physBank < psram_banks) {
			return buffer_psram + physAddr;
		} else if(physBank == NILE_SEG_ROM_BOOT || physBank == NILE_SEG_ROM_BOOT_PCV2) {
			return _prgRom + (physAddr & (_prgRomSize - 1));
		} else if(physBank == NILE_SEG_ROM_RX && (!write || isDebugger)) {
			return buffer_spi_rx[GetSpiBankIndex(true)] + (physAddr & (NILE_SPI_SIZE - 1));
		}
	}

	return NULL;
}

bool WsCartNileswan::InternalReadCart(uint32_t addr, uint8_t &value)
{
	if(addr < 0x10000) {
		return false;
	}

	bool inSram = (addr & 0xF0000) == 0x10000;
	uint8_t* buffer = ResolveBank(addr, false, false);

	uint8_t cpuBank = (addr >> 16) & 0xF;
	if((_nstate.EmuCnt & NILE_EMU_FLASH_FSM) && _state.RomInRamBank && inSram) {
		if(wwState == WwState::FAST) {
			value = 0x00;
			return true;
		}
		if(wwState == WwState::ERASE) {
			value = 0xFF;
			return true;
		}
	}
	
	if(buffer != NULL) {
    	if((_nstate.EmuCnt & NILE_EMU_LODSW_TRICK) && (cpuBank == 2 || cpuBank == 3)) {
    		bool oldFlashEnable = _state.RomInRamBank;
    		_state.RomInRamBank = true;
    		uint8_t* writeBuffer = ResolveBank((addr & 0xFFFF) | 0x10000, false, false);
    		_state.RomInRamBank = oldFlashEnable;
    
    		if(writeBuffer) {
    		    *writeBuffer = *buffer;		
    		} else {
                printf("nileswan: BUG unmapped memory during LODSW trick at %05X\n", addr);
            }
    	}

		value = *buffer;
	} else {
		value = 0xFF;
	}
	return true;
}

bool WsCartNileswan::InternalWriteCart(uint32_t addr, uint8_t value)
{
	if(addr < 0x10000) {
		return false;
	}

	bool inSram = (addr & 0xF0000) == 0x10000;
	uint8_t* buffer = ResolveBank(addr, true, false);

	if((_nstate.EmuCnt & NILE_EMU_FLASH_FSM) && _state.RomInRamBank && inSram) {
		if(wwState == WwState::READ) {
			if(value == 0xAA) {
				wwState = WwState::UNLOCK_1;
			} else {
				wwState = WwState::READ;
			}
		} else if(wwState == WwState::UNLOCK_1) {
			if(value == 0x55) {
				wwState = WwState::UNLOCK_2;
			} else {
				wwState = WwState::READ;
			}
		} else if(wwState == WwState::UNLOCK_2) {
			if(value == 0x20) {
				wwState = WwState::FAST;
			} else if(value == 0xA0) {
				wwState = WwState::WRITE;
			} else if(value == 0x10) {
				wwState = WwState::ERASE;
			} else if(value == 0x30) {
				wwState = WwState::ERASE;
			} else {
				wwState = WwState::READ;
			}
		} else if(wwState == WwState::FAST) {
			if(value == 0xA0) {
				wwState = WwState::FAST_WRITE;
			} else if(value == 0x90) {
				wwState = WwState::READ; /* Reset mode */
			} else {
				wwState = WwState::FAST;
			}
		} else if(wwState == WwState::FAST_WRITE) {
		    if(buffer != NULL) {
    			*buffer = value;
    		}
			wwState = WwState::FAST;
		} else if(wwState == WwState::WRITE) {
    		if(buffer != NULL) {
    			*buffer = value;
    		}
			wwState = WwState::READ;
		} else if(wwState == WwState::ERASE) {
			if(value == 0xAA) {
				wwState = WwState::UNLOCK_1;
			} else {
				wwState = WwState::READ;
			}
		}
		return true;
	}

	if(buffer != NULL) {
		*buffer = value;
	}
	return true;
}

void WsCartNileswan::Serialize(Serializer& s)
{
	WsCart::Serialize(s);

	SV(_nstate.PowCnt);
	SV(_nstate.EmuCnt);
	SV(_nstate.SpiCnt);
	SV(_nstate.BankMask);
	SV(_nstate.FpgaCore);
	SV(_nstate.IrqStatus);
	SV(_nstate.IrqEnable);
	SV(wwState);

	SVArray(buffer_psram, psram_banks * 0x10000);
	SVArray(buffer_sram, sram_banks * 0x10000);
	SVArray(buffer_ipc, NILE_IPC_SIZE);
	SVArray(buffer_spi_tx[0], NILE_SPI_SIZE);
	SVArray(buffer_spi_tx[1], NILE_SPI_SIZE);
	SVArray(buffer_spi_rx[0], NILE_SPI_SIZE);
	SVArray(buffer_spi_rx[1], NILE_SPI_SIZE);

	SV(flash);
	SV(mcu);
	SV(tf);
}
