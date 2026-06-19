#include "Shared/MemoryType.h"
#include "WS/Carts/WsRtc.h"
#include "WS/Carts/Nileswan/WsCartNileswan.h"
#include "pch.h"
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
    nile_fpga_core = -1;
    FpgaReset();

    flash.Reset();
    mcu.Reset(false);
    tf.Reset();
}

void WsCartNileswan::FpgaReset()
{
    _state.SelectedBanks[2] = 0xFF;
	_state.SelectedBanks[3] = 0xFF;
	_state.SelectedBanks[0] = 0xFF;
	_state.SelectedBanks[1] = 0xFF;
	_state.ExtSelectedBanks[2] = 0xFF;
	_state.ExtSelectedBanks[3] = 0xFF;
	_state.ExtSelectedBanks[0] = 0x00;
	_state.ExtSelectedBanks[1] = 0xFF;
	_state.RomInRamBank = false;
	nile_spi_cnt = 0;
	nile_pow_cnt = NILE_POW_UNLOCK;
	nile_bank_mask = 0xFFFF;
	nile_emu_cnt = 0;
	nile_ww_state = WW_STATE_READ;

	memset(buffer_ipc, 0, sizeof(buffer_ipc));
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

bool WsCartNileswan::IsTFPowered()
{
    return nile_pow_cnt & NILE_POW_TF;
}

// I/O handling

void WsCartNileswan::OnSpiCntUpdate(uint16_t prev_spi_cnt)
{
	if(!(nile_spi_cnt & NILE_SPI_390KHZ) && !(nile_pow_cnt & NILE_POW_CLOCK)) {
		return;
	}

	if((nile_spi_cnt & NILE_SPI_DEV_MASK) != NILE_SPI_DEV_FLASH) {
		flash.Reset();
	}

	if(nile_spi_cnt & NILE_SPI_BUSY) {
		const char* device_name = "none";
		if((nile_spi_cnt & NILE_SPI_DEV_MASK) == NILE_SPI_DEV_TF) {
			device_name = "TF card";
		} else if((nile_spi_cnt & NILE_SPI_DEV_MASK) == NILE_SPI_DEV_FLASH) {
			device_name = "SPI flash";
		} else if((nile_spi_cnt & NILE_SPI_DEV_MASK) == NILE_SPI_DEV_MCU) {
			device_name = "MCU";
		}

		uint8_t* tx_buffer = buffer_spi_tx[GetSpiBankIndex(false)];
		uint8_t* rx_buffer = buffer_spi_rx[GetSpiBankIndex(false)];
		uint32_t length = (nile_spi_cnt & 0x1FF) + 1;
		uint32_t pos = 0;
		uint16_t mode = nile_spi_cnt & NILE_SPI_MODE_MASK;
		if(mode == NILE_SPI_MODE_WAIT_READ) {
			int32_t timeout = 8192;
			uint32_t bytes_skipped = 0;
			while(--timeout) {
				if((rx_buffer[0] = SpiExchange(0xFF)) != 0xFF) {
					break;
				}
				bytes_skipped++;
			}
			if(timeout <= 0) {
				printf("nileswan/spi: !!! WAIT_READ timeout for %s !!!\n", device_name);
				return;
			} else if(bytes_skipped > 0) {
				printf("nileswan/spi: skipped %d bytes\n", bytes_skipped);
			}
			pos++;
		}
		bool mode_reads = mode != NILE_SPI_MODE_WRITE;
		bool mode_writes = mode == NILE_SPI_MODE_WRITE || mode == NILE_SPI_MODE_EXCH;
		for(; pos < length; pos++) {
			uint8_t rx = SpiExchange(mode_writes ? tx_buffer[pos] : 0xFF);
			if(mode_reads) {
				rx_buffer[pos] = rx;
			}
		}
		printf("nileswan/spi: %s %d bytes %s %s",
			mode_reads ? (mode_writes ? "exchanging" : "reading") : (mode_writes ? "writing" : "???"),
			length,
			mode_reads ? (mode_writes ? "with" : "from") : (mode_writes ? "to" : "with"),
			device_name);
		if(mode_writes) {
			printf(" [%02x", tx_buffer[0]);
			for(pos = 1; pos < length; pos++) {
				printf(" %02x", tx_buffer[pos]);
			}
			printf("]");
		}
		if(mode_reads) {
			printf(" [%02x", rx_buffer[0]);
			for(pos = 1; pos < length; pos++) {
				printf(" %02x", rx_buffer[pos]);
			}
			printf("]");
		}
		printf("\n");
		nile_spi_cnt = nile_spi_cnt & ~NILE_SPI_BUSY;
	}
	fflush(stdout);
}

void WsCartNileswan::OnPowCntUpdate(uint8_t new_value)
{
	uint8_t old_value = nile_pow_cnt;
	nile_pow_cnt = new_value;
	if(!(nile_pow_cnt & NILE_POW_TF)) {
		tf.Reset();
	}
	if(!(old_value & NILE_POW_MCU_RESET) && (new_value & NILE_POW_MCU_RESET)) {
		bool bootloader_mode = (new_value & NILE_POW_MCU_BOOT0) != 0;
		printf("nileswan/mcu: reset, in %s mode\n", bootloader_mode ? "bootloader" : "native");
		mcu.Reset(bootloader_mode);
	}
}


uint8_t WsCartNileswan::ReadPort(uint16_t index)
{
	switch(index) {
	    case 0xCA:
		case 0xCB:
		    return _rtc->ReadPort(index);
		case IO_CART_FLASH:
			return _state.RomInRamBank ? 1 : 0;
		case IO_BANK_ROM_LINEAR:
			return _state.SelectedBanks[0];
		case IO_BANK_2003_ROM_LINEAR:
			return _state.SelectedBanks[0];
		case IO_BANK_RAM:
			return _state.SelectedBanks[1];
		case IO_BANK_2003_RAM:
			return _state.SelectedBanks[1];
		case IO_BANK_2003_RAM + 1:
			return _state.ExtSelectedBanks[1];
		case IO_BANK_ROM0:
			return _state.SelectedBanks[2];
		case IO_BANK_2003_ROM0:
			return _state.SelectedBanks[2];
		case IO_BANK_2003_ROM0 + 1:
			return _state.ExtSelectedBanks[2];
		case IO_BANK_ROM1:
			return _state.SelectedBanks[3];
		case IO_BANK_2003_ROM1:
			return _state.SelectedBanks[3];
		case IO_BANK_2003_ROM1 + 1:
			return _state.ExtSelectedBanks[3];
		case IO_NILE_POW_CNT:
			return nile_pow_cnt;
		case IO_NILE_SEG_MASK:
			return nile_bank_mask;
		case IO_NILE_SEG_MASK + 1:
			return nile_bank_mask >> 8;
		case IO_NILE_SPI_CNT:
			return nile_spi_cnt;
		case IO_NILE_SPI_CNT + 1:
			return nile_spi_cnt >> 8;
		case IO_NILE_EMU_CNT:
			return nile_emu_cnt;
		case IO_NILE_BOARD_REVISION:
			return NILE_EMULATED_BOARD_REVISION;
	}
	return 0x00;
}

void WsCartNileswan::WritePort(uint16_t index, uint8_t value)
{
	switch(index) {
	    case 0xCA:
		case 0xCB:
    		if(nile_pow_cnt & NILE_POW_IO_2003) {
                _rtc->WritePort(index, value);
    		}
            break;
		case IO_CART_FLASH:
			if(!(nile_pow_cnt & NILE_POW_IO_2003)) {
				break;
			}
			_state.RomInRamBank = value & 0x01;
			break;
		case IO_BANK_ROM_LINEAR:
			_state.SelectedBanks[0] = value;
			break;
		case IO_BANK_2003_ROM_LINEAR:
			if(!(nile_pow_cnt & NILE_POW_IO_2003)) {
				break;
			}
			_state.SelectedBanks[0] = value;
			break;
		case IO_BANK_RAM:
			_state.SelectedBanks[1] = value;
			break;
		case IO_BANK_2003_RAM:
			if(!(nile_pow_cnt & NILE_POW_IO_2003)) {
				break;
			}
			_state.SelectedBanks[1] = value;
			break;
		case IO_BANK_2003_RAM + 1:
			if(!(nile_pow_cnt & NILE_POW_IO_2003)) {
				break;
			}
			_state.ExtSelectedBanks[1] = value;
			break;
		case IO_BANK_ROM0:
			_state.SelectedBanks[2] = value;
			break;
		case IO_BANK_2003_ROM0:
			if(!(nile_pow_cnt & NILE_POW_IO_2003)) {
				break;
			}
			_state.SelectedBanks[2] = value;
			break;
		case IO_BANK_2003_ROM0 + 1:
			if(!(nile_pow_cnt & NILE_POW_IO_2003)) {
				break;
			}
			_state.ExtSelectedBanks[2] = value;
			break;
		case IO_BANK_ROM1:
			_state.SelectedBanks[3] = value;
			break;
		case IO_BANK_2003_ROM1:
			if(!(nile_pow_cnt & NILE_POW_IO_2003)) {
				break;
			}
			_state.SelectedBanks[3] = value;
			break;
		case IO_BANK_2003_ROM1 + 1:
			if(!(nile_pow_cnt & NILE_POW_IO_2003)) {
				break;
			}
			_state.ExtSelectedBanks[3] = value;
			break;
		case IO_NILE_POW_CNT:
			if(!(nile_pow_cnt & NILE_POW_IO_NILE) && value != NILE_POW_UNLOCK) {
				break;
			}
			OnPowCntUpdate(value);
			break;
		case IO_NILE_WARMBOOT_CNT:
			nile_fpga_core = value & 0x3;
			printf("nileswan/fpga: warmboot to core %d\n", nile_fpga_core);
			FpgaReset();
			break;
		case IO_NILE_SEG_MASK:
			if(!(nile_pow_cnt & NILE_POW_IO_NILE)) {
				break;
			}
			nile_bank_mask = (nile_bank_mask & 0xFF00) | value;
			break;
		case IO_NILE_SEG_MASK + 1:
			if(!(nile_pow_cnt & NILE_POW_IO_NILE)) {
				break;
			}
			nile_bank_mask = (nile_bank_mask & 0xFF) | (value << 8);
			break;
		case IO_NILE_SPI_CNT: {
			if(!(nile_pow_cnt & NILE_POW_IO_NILE)) {
				break;
			}
			uint16_t new_nile_spi_cnt = (nile_spi_cnt & 0xFF00) | value;
			if(nile_spi_cnt & NILE_SPI_BUSY) {
				printf("nileswan/spi: BUG trying to write to SPI control while transfer active (control %04X => %04X)\n",
					nile_spi_cnt,
					new_nile_spi_cnt);
				break;
			}
			nile_spi_cnt = new_nile_spi_cnt;
		} break;
		case IO_NILE_SPI_CNT + 1: {
			if(!(nile_pow_cnt & NILE_POW_IO_NILE)) {
				break;
			}
			uint16_t new_nile_spi_cnt = (nile_spi_cnt & 0xFF) | (value << 8);
			if(nile_spi_cnt != new_nile_spi_cnt && (new_nile_spi_cnt | NILE_SPI_BUSY) == nile_spi_cnt) {
				printf("nileswan/spi: abort\n");
			} else if(nile_spi_cnt & NILE_SPI_BUSY) {
				printf("nileswan/spi: BUG trying to write to SPI control while transfer active (control %04X => %04X)\n",
					nile_spi_cnt,
					new_nile_spi_cnt);
				break;
			}
			uint16_t old_spi_cnt = nile_spi_cnt;
			nile_spi_cnt = new_nile_spi_cnt;
			printf("nileswan/spi: control = %04X\n", nile_spi_cnt);
			OnSpiCntUpdate(old_spi_cnt);
		} break;
		case IO_NILE_EMU_CNT:
			if(!(nile_pow_cnt & NILE_POW_IO_NILE)) {
				break;
			}
			nile_emu_cnt = value & 0x3F;
			break;
	}
	
	RefreshMappings();
}

// SPI routing

int WsCartNileswan::GetSpiBankIndex(bool is_swan)
{
	bool is_back_buffer = (nile_spi_cnt & NILE_SPI_BUFFER_IDX) != 0;
	is_back_buffer ^= is_swan;
	return is_back_buffer ? 0 : 1;
}

uint8_t WsCartNileswan::SpiExchange(uint8_t tx)
{
    if((nile_spi_cnt & NILE_SPI_DEV_MASK) == NILE_SPI_DEV_TF) {
		return tf.SpiExchange(tx);
	} else if((nile_spi_cnt & NILE_SPI_DEV_MASK) == NILE_SPI_DEV_FLASH) {
		return flash.SpiExchange(tx);
	} else if((nile_spi_cnt & NILE_SPI_DEV_MASK) == NILE_SPI_DEV_MCU) {
		return mcu.SpiExchange(tx);
	} else {
		return 0xFF;
	}
}

// Memory routing

MemoryType WsCartNileswan::GetMemoryTypeForBank(uint16_t bank, bool rom, bool linear)
{
    if(rom && linear) {
        bank &= 0x1F;
        if(bank == (NILE_SEG_ROM_BOOT >> 4)) {
            return MemoryType::WsNileBootrom;         
        } else if(bank < (psram_banks >> 4)) {
            return MemoryType::WsPrgRom;   
        }
    } else if(rom) {
        bank &= 0x1FF;
        if(bank == NILE_SEG_ROM_BOOT || bank == NILE_SEG_ROM_BOOT_PCV2) {
            return MemoryType::WsNileBootrom;
        /* } else if(bank == NILE_SEG_ROM_RX) {
            return MemoryType::WsNileSpiRx; */
        } else if(bank < psram_banks) {
            return MemoryType::WsPrgRom;
        }
    } else {
        bank &= 0xF;
        if(bank == NILE_SEG_RAM_IPC) {
            return MemoryType::WsNileIpc;
        /* } else if(bank == NILE_SEG_RAM_TX) {
            return MemoryType::WsNileSpiTx; */            
        } else if(bank < sram_banks) {
            return MemoryType::WsCartRam;
        }
    }
    return MemoryType::None;
}

uint32_t WsCartNileswan::GetSelectedBank(uint8_t index)
{
	if(index == 0) {
		return _state.SelectedBanks[0];
	} else {
		return (_state.ExtSelectedBanks[index - 1] << 8) | _state.SelectedBanks[index];
	}
}

AddressInfo WsCartNileswan::GetAbsoluteAddress(uint32_t relAddr)
{
    uint8_t* ptr;
	ResolveBank(relAddr, &ptr, false, false);

	if(ptr >= buffer_psram && ptr < buffer_psram + (psram_banks * 0x10000)) {
		return { (int)(ptr - buffer_psram), MemoryType::WsPrgRom };
	} else if(ptr >= buffer_sram && ptr < buffer_sram + (sram_banks * 0x10000)) {
		return { (int)(ptr - buffer_sram), MemoryType::WsCartRam };
	} else if(ptr >= _prgRom && ptr < _prgRom + _prgRomSize) {
		return { (int)(ptr - _prgRom), MemoryType::WsNileBootrom };
	} else if(ptr >= buffer_ipc && ptr < buffer_ipc + NILE_IPC_SIZE) {
		return { (int)(ptr - buffer_ipc), MemoryType::WsNileIpc };
	/* } else if(ptr >= (uint8_t*)buffer_spi_tx && ptr < (uint8_t*)buffer_spi_tx + NILE_SPI_SIZE) {
		return { (int)(ptr - (uint8_t*)buffer_spi_tx), MemoryType::WsNileSpiTx };
	} else if(ptr >= (uint8_t*)buffer_spi_rx && ptr < (uint8_t*)buffer_spi_rx + NILE_SPI_SIZE) {
		return { (int)(ptr - (uint8_t*)buffer_spi_rx), MemoryType::WsNileSpiRx }; */
	}
	
	return { -1, MemoryType::None };
}

void WsCartNileswan::RefreshMappings()
{
	WsCart::RefreshMappings();
	
	Map(0x10000, 0x1FFFF, GetMemoryTypeForBank(GetSelectedBank(1), _state.RomInRamBank, false), GetSelectedBank(1) * 0x10000, true);
	Map(0x20000, 0x2FFFF, GetMemoryTypeForBank(GetSelectedBank(2), true, false), GetSelectedBank(2) * 0x10000, true);
	Map(0x30000, 0x3FFFF, GetMemoryTypeForBank(GetSelectedBank(3), true, false), GetSelectedBank(3) * 0x10000, true);
	Map(0x40000, 0xFFFFF, GetMemoryTypeForBank(GetSelectedBank(0), true, true), GetSelectedBank(0) * 0x100000 + 0x40000, true);
	
	_hasCustomReadHandler = true;
	_hasCustomWriteHandler = true;
}

void WsCartNileswan::ResolveBank(uint32_t address, uint8_t** buffer, bool write, bool is_debugger)
{
	uint8_t cpu_bank = (address >> 16) & 0xF;
	bool is_ram = (cpu_bank == 1) && !_state.RomInRamBank;

	uint32_t physical_bank;
	uint16_t mask_bit;
	if(cpu_bank == 1) {
		physical_bank = GetSelectedBank(1);
		mask_bit = NILE_SEG_SRAM_LOCK;
	} else if(cpu_bank == 2) {
		physical_bank = GetSelectedBank(2);
		mask_bit = NILE_SEG_ROM0_LOCK;
	} else if(cpu_bank == 3) {
		physical_bank = GetSelectedBank(3);
		mask_bit = NILE_SEG_ROM1_LOCK;
	} else {
		physical_bank = (_state.SelectedBanks[0] << 4) | cpu_bank;
		mask_bit = 0;
	}

	*buffer = NULL;

	if(is_ram) {
		if(!mask_bit || (nile_bank_mask & mask_bit)) {
			physical_bank &= (nile_bank_mask >> 12);
		} else {
			physical_bank &= 0xF;
		}
		uint32_t physical_address = (physical_bank << 16) | (address & 0xFFFF);

		if(physical_bank < sram_banks) {
			if(nile_pow_cnt & NILE_POW_SRAM) {
				if(nile_emu_cnt & NILE_EMU_SRAM_32KB) {
					physical_address &= ~0x8000;
				}
				*buffer = buffer_sram + physical_address;
			}
		} else if(physical_bank == NILE_SEG_RAM_IPC) {
			*buffer = buffer_ipc + (physical_address & (NILE_IPC_SIZE - 1));
		} else if(physical_bank == NILE_SEG_RAM_TX && (write || is_debugger)) {
		    *buffer = buffer_spi_tx[GetSpiBankIndex(true)] + (physical_address & (NILE_SPI_SIZE - 1));
		}
	} else {
		if(!mask_bit || (nile_bank_mask & mask_bit)) {
			physical_bank &= (nile_bank_mask & 0x1FF);
		} else {
			physical_bank &= 0x1FF;
		}
		uint32_t physical_address = (physical_bank << 16) | (address & 0xFFFF);

		if(physical_bank < psram_banks) {
			*buffer = buffer_psram + physical_address;
		} else if(physical_bank == NILE_SEG_ROM_BOOT || physical_bank == NILE_SEG_ROM_BOOT_PCV2) {
			*buffer = _prgRom + (physical_address & (_prgRomSize - 1));
		} else if(physical_bank == NILE_SEG_ROM_RX && (!write || is_debugger)) {
			*buffer = buffer_spi_rx[GetSpiBankIndex(true)] + (physical_address & (NILE_SPI_SIZE - 1));
		}
	}
}

bool WsCartNileswan::InternalReadCart(uint32_t addr, uint8_t &value)
{
	if(addr < 0x10000) {
		return false;
	}
	
    uint8_t* buffer;
    bool in_sram = (addr & 0xF0000) == 0x10000;
	ResolveBank(addr, &buffer, false, false);

	uint8_t cpu_bank = (addr >> 16) & 0xF;
	if((cpu_bank == 2 || cpu_bank == 3) && (nile_emu_cnt & 0x20)) {
		uint8_t* write_buffer;

		bool old_flash_enable = _state.RomInRamBank;
		_state.RomInRamBank = true;
		ResolveBank((addr & 0xFFFF) | 0x10000, &write_buffer, false, false);
		_state.RomInRamBank = old_flash_enable;

		*write_buffer = *buffer;
	}

	if((nile_emu_cnt & NILE_EMU_FLASH_FSM) && _state.RomInRamBank && in_sram) {
		if(nile_ww_state == WW_STATE_FAST) {
			value = 0x00;
			return true;
		}
		if(nile_ww_state == WW_STATE_ERASE) {
			value = 0xFF;
			return true;
		}
	}
	if(buffer != NULL) {
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
	
    uint8_t* buffer;
    bool in_sram = (addr & 0xF0000) == 0x10000;
	ResolveBank(addr, &buffer, true, false);

	if((nile_emu_cnt & NILE_EMU_FLASH_FSM) && _state.RomInRamBank && in_sram) {
		if(nile_ww_state == WW_STATE_READ) {
			if(value == 0xAA) {
				nile_ww_state = WW_STATE_UNLOCK_1;
			} else {
				nile_ww_state = WW_STATE_READ;
			}
		} else if(nile_ww_state == WW_STATE_UNLOCK_1) {
			if(value == 0x55) {
				nile_ww_state = WW_STATE_UNLOCK_2;
			} else {
				nile_ww_state = WW_STATE_READ;
			}
		} else if(nile_ww_state == WW_STATE_UNLOCK_2) {
			if(value == 0x20) {
				nile_ww_state = WW_STATE_FAST;
			} else if(value == 0xA0) {
				nile_ww_state = WW_STATE_WRITE;
			} else if(value == 0x10) {
				nile_ww_state = WW_STATE_ERASE;
			} else if(value == 0x30) {
				nile_ww_state = WW_STATE_ERASE;
			} else {
				nile_ww_state = WW_STATE_READ;
			}
		} else if(nile_ww_state == WW_STATE_FAST) {
			if(value == 0xA0) {
				nile_ww_state = WW_STATE_FAST_WRITE;
			} else if(value == 0x90) {
				nile_ww_state = WW_STATE_READ; /* Reset mode */
			} else {
				nile_ww_state = WW_STATE_FAST;
			}
		} else if(nile_ww_state == WW_STATE_FAST_WRITE) {
			*buffer = value;
			nile_ww_state = WW_STATE_FAST;
		} else if(nile_ww_state == WW_STATE_WRITE) {
			*buffer = value;
			nile_ww_state = WW_STATE_READ;
		} else if(nile_ww_state == WW_STATE_ERASE) {
			if(value == 0xAA) {
				nile_ww_state = WW_STATE_UNLOCK_1;
			} else {
				nile_ww_state = WW_STATE_READ;
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

	// TODO
}
