#include "pch.h"
#include "Utilities/Serializer.h"
#include "WS/Carts/Nileswan/NileTF.h"
#include "WS/Carts/Nileswan/WsCartNileswan.h"
#include "WS/Carts/Nileswan/hardware.h"

NileTF::NileTF(WsCartNileswan* _parent)
	: parent(_parent), txBuf(1024), rxBuf(1024)
{
}

NileTF::~NileTF()
{
	if(file != nullptr) {
		fclose(file);
	}
}

void NileTF::Reset()
{
	txBuf.Reset();
	rxBuf.Reset();
	status = 0;
	is_acmd = false;
	reading = false;
	writing = 0;
}

#define SPI_TF_WRITING_SINGLE 1
#define SPI_TF_WRITING_MULTIPLE 2

#define TF_ILLEGAL_COMMAND 0x04
#define TF_PARAMETER_ERROR 0x40

uint8_t NileTF::SpiExchange(uint8_t tx)
{
	uint8_t rx;
	uint8_t response[1024];

	if(!parent->IsTFPowered()) {
		return 0xFF;
	}

	if(rxBuf.pos || writing || tx < 0x80) {
		rxBuf.Push(&tx, 1);
	}
	txBuf.Pop(&rx, 1);

	if(writing) {
		// handle card output
		if(rx != 0xFF) {
			return rx;
		}

		// remove stall bytes
		while(rxBuf.data[0] == 0xFF && rxBuf.pos) {
			rxBuf.Pop(NULL, 1);
		}

		// data token present?
		if(!rxBuf.pos) {
			return 0xFF;
		}

		if(writing == SPI_TF_WRITING_SINGLE) {
			if(rxBuf.data[0] != 0xFE) {
				printf("nileswan/spi/tf: unexpected data block start %02x\n", rxBuf.data[0]);
				writing = 0;
				rxBuf.Pop(NULL, 1);
				return 0xFF;
			}
			// write data block
			if(rxBuf.pos < 515) {
				return 0xFF;
			}
			if(!feof(file)) {
				fwrite(rxBuf.data + 1, 512, 1, file);
			}
			writing = 0;
			rxBuf.Pop(NULL, 515);

			response[0] = 0xE5;
			txBuf.Push(response, 1);
		}

		if(writing == SPI_TF_WRITING_MULTIPLE) {
			if(rxBuf.data[0] != 0xFC) {
				if(rxBuf.data[0] != 0xFD) {
					printf("nileswan/spi/tf: unexpected data block start %02x\n", rxBuf.data[0]);
				}
				writing = 0;
				rxBuf.Pop(NULL, 1);
				return 0xFF;
			}
			// write data block
			if(rxBuf.pos < 515) {
				return 0xFF;
			}
			if(!feof(file)) {
				fwrite(rxBuf.data + 1, 512, 1, file);
			}
			rxBuf.Pop(NULL, 515);

			response[0] = 0xE5;
			txBuf.Push(response, 1);
		}
	}

	if(reading && !txBuf.pos) {
		response[0] = 0xFE;
		for(int i = 0; i < 512; i++) {
			response[1 + i] = file != NULL ? fgetc(file) : i;
		}
		// TODO: CRC
		txBuf.Push(response, 515);
	}

	while(rxBuf.pos >= 6) {
		while(rxBuf.data[0] >= 0x80 && rxBuf.pos) {
			rxBuf.Pop(NULL, 1);
		}
		if(rxBuf.pos < 6) {
			break;
		}

		uint8_t cmd = rxBuf.data[0];
		uint32_t arg =
			(rxBuf.data[1] << 24) |
			(rxBuf.data[2] << 16) |
			(rxBuf.data[3] << 8) |
			rxBuf.data[4];
		uint32_t response_length = 1;
		response[0] = 0;
		if(is_acmd) {
			is_acmd = false;
			switch(cmd & 0x3F) {
				case 23:
					printf("nileswan/spi/tf: set block count = %d (acmd)\n", arg);
					status = 0x00;
					break;
				case 41:
					printf("nileswan/spi/tf: init (acmd)\n");
					status = 0x00;
					break;
				default:
					printf("nileswan/spi/tf: unknown acommand %d\n", cmd & 0x3F);
					response[0] |= TF_ILLEGAL_COMMAND;
					break;
			}
		} else {
			switch(cmd & 0x3F) {
				case 0:
					printf("nileswan/spi/tf: reset\n");
					status = 0x01;
					break;
				case 1:
					printf("nileswan/spi/tf: init\n");
					status = 0x00;
					break;
				case 8:
					printf("nileswan/spi/tf: read interface configuration\n");
					response[1] = 0;
					response[2] = 0;
					response[3] = 0x1;
					response[4] = arg & 0xFF;
					response_length = 5;
					break;
				case 9: {
					size_t file_size;
					{
						size_t cur_pos = ftell(file);
						fseek(file, 0, SEEK_END);
						file_size = ftell(file);
						fseek(file, cur_pos, SEEK_SET);
					}
					printf("nileswan/spi/tf: read csd\n");
					response[0] = 0x00;
					response[1] = 0xFE;
					// FIXME: populate full CSD
					memset(response + 2, 0, 16);
					response[2] = 0x40;
					response[5] = 0x32;
					uint32_t csd_size = (file_size + 524287) / 524288;
					printf("%d\n", csd_size);
					response[11] = csd_size;
					response[10] = csd_size >> 8;
					response[9] = (csd_size >> 16) & 0x3f;
					response_length = 18;
				} break;
				case 12:
					printf("nileswan/spi/tf: stop reading\n");
					reading = false;
					txBuf.pos = 0;
					response[0] = 0xFF; // skipped byte
					response[1] = 0xFF; // command processing delay
					response[2] = 0x00; // command response
					memset(response + 3, 0, NILE_TF_STOP_TRANSFER_BUSY_DELAY_BYTES);
					response[3 + NILE_TF_STOP_TRANSFER_BUSY_DELAY_BYTES] = 0xFF;
					response_length = 4 + NILE_TF_STOP_TRANSFER_BUSY_DELAY_BYTES;
					break;
				case 16:
					printf("nileswan/spi/tf: set block length = %d\n", arg);
					if(arg != 512) {
						response[0] |= TF_PARAMETER_ERROR;
					}
					break;
				case 17:
				case 18: {
					printf("nileswan/spi/tf: reading %s @ %08X\n",
						(cmd & 0x3F) == 18 ? "multiple sectors" : "single sector",
						arg);
					int data_ofs = NILE_TF_DATA_BLOCK_READ_DELAY_BYTES;
					response_length = data_ofs + 515;
					memset(response + 1, 0xFF, response_length - 1);
					if(file != NULL) {
						fseek(file, arg, SEEK_SET);
					}
					response[data_ofs] = 0xFE;
					for(int i = 0; i < 512; i++) {
						response[data_ofs + 1 + i] = file != NULL ? fgetc(file) : i;
					}
					// TODO: CRC
					if((cmd & 0x3F) == 18) {
						reading = true;
					}
				} break;
				case 24:
				case 25: {
					printf("nileswan/spi/tf: writing %s @ %08X\n",
						(cmd & 0x3F) == 25 ? "multiple sectors" : "single sector",
						arg);
					if(file != NULL) {
						fseek(file, arg, SEEK_SET);
					}
					writing = (cmd & 0x3F) == 25 ? SPI_TF_WRITING_MULTIPLE : SPI_TF_WRITING_SINGLE;
				} break;
				case 55: {
					is_acmd = true;
				} break;
				case 58: {
					printf("nileswan/spi/tf: read ocr\n");
					response[1] = 0x80;
					response[2] = 0xFF;
					response[3] = 0x80;
					response[4] = 0x00;
					response_length = 5;
				} break;
				default:
					printf("nileswan/spi/tf: unknown command %d\n", cmd & 0x3F);
					response[0] |= TF_ILLEGAL_COMMAND;
					break;
			}
		}
		rxBuf.Pop(NULL, 6);
		response[0] |= status;
		txBuf.Push(response, response_length);
	}
	return rx;
}

void NileTF::Serialize(Serializer& s)
{
    SV(rxBuf);
    SV(txBuf);
    
	SV(status);
	SV(is_acmd);
	SV(reading);
	SV(writing);

	uint32_t filePos = ftell(file);
	SV(filePos);
	if(!s.IsSaving()) {
		fseek(file, filePos, SEEK_SET);
	}
}
