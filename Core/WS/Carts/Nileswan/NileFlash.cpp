#include "WS/Carts/Nileswan/NileFlash.h"
#include "WS/Carts/Nileswan/hardware.h"
#include "pch.h"

NileFlash::NileFlash()
	: txBuf(1024), rxBuf(1024)
{
    sr1 = 0;
    sr2 = 0;
    sr3 = 0;
    sleeping = false;
    position = 0;
}

NileFlash::~NileFlash()
{
    if (file != nullptr)
        fclose(file);
}

void NileFlash::Reset()
{
    txBuf.Reset();
    rxBuf.Reset();
    mode = 0;
}

static const uint8_t spi_flash_mfr_id = 0xEF;
static const uint8_t spi_flash_dev_id = 0x14;
static const uint8_t spi_flash_jedec_id[] = { spi_flash_mfr_id, 0x40, 0x15 };
static const uint8_t spi_flash_uuid[] = { 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF };

uint8_t NileFlash::SpiExchange(uint8_t tx)
{
    uint8_t rx = 0xFF;
    uint32_t size = 0;

    if (mode == NILE_FLASH_CMD_READ) {
        if (file != NULL)
            rx = feof(file) ? 0xFF : fgetc(file);
        else
            rx = 0x90;
        position++;
    } else if (mode == NILE_FLASH_CMD_WRITE) {
        // TODO: only set bits one way
        // TODO: handle WEL
        fseek(file, position, SEEK_SET);
        fputc(tx, file);

        // increment within page
        position = ((position + 1) & 0xFF) | (position & 0xFFFFFF00);
    } else if (mode == NILE_FLASH_CMD_RDSR1) {
        return sr1;
    } else if (mode == NILE_FLASH_CMD_RDSR2) {
        return sr2;
    } else if (mode == NILE_FLASH_CMD_RDSR3) {
        return sr3;
    } else {
        rxBuf.Push(&tx, 1);
        txBuf.Pop(&rx, 1);

        if (sleeping && rxBuf.data[0] != NILE_FLASH_CMD_WAKE_ID) {
            printf("nileswan/spi/flash: !! byte %02X sent while asleep !!\n", rxBuf.data[0]);
            return 0xFF;
        }

        switch (rxBuf.data[0]) {
            case NILE_FLASH_CMD_ERASE_4K:
                if (!size) size = 4096;
            case NILE_FLASH_CMD_ERASE_32K:
                if (!size) size = 32768;
            case NILE_FLASH_CMD_ERASE_64K:
                if (!size) size = 65536;
            case NILE_FLASH_CMD_WRITE:
            case NILE_FLASH_CMD_READ:
                if (rxBuf.pos >= 4) {
                    mode = rxBuf.data[0];
                    position =
                        (rxBuf.data[1] << 16) |
                        (rxBuf.data[2] << 8) |
                        rxBuf.data[3];
                    rxBuf.Pop(NULL, 4);
                    if (file != NULL)
                        fseek(file, position, SEEK_SET);
                    if (mode == NILE_FLASH_CMD_WRITE) printf("nileswan/spi/flash: write starting at location %06X\n", position);
                    else if (mode == NILE_FLASH_CMD_READ) printf("nileswan/spi/flash: read starting at location %06X\n", position);
                    else printf("nileswan/spi/flash: erasing %d bytes at location %06X\n", size, position);
                }
                break;
            case NILE_FLASH_CMD_RDSR1:
            case NILE_FLASH_CMD_RDSR2:
            case NILE_FLASH_CMD_RDSR3:
                mode = rxBuf.data[0];
                rxBuf.Pop(NULL, 1);
                break;
            case NILE_FLASH_CMD_RDUUID:
                rxBuf.Pop(NULL, 1);
                txBuf.Push(NULL, 4);
                txBuf.Push(spi_flash_uuid, 8);
                break;
            case NILE_FLASH_CMD_MFR_ID:
                if (rxBuf.pos >= 4) {
                    rxBuf.Pop(NULL, 4);
                    txBuf.Push(&spi_flash_mfr_id, 1);
                    txBuf.Push(&spi_flash_dev_id, 1);
                }
                break;
            case NILE_FLASH_CMD_RDID:
                rxBuf.Pop(NULL, 1);
                txBuf.Push(spi_flash_jedec_id, 3);
                break;
            case NILE_FLASH_CMD_WAKE_ID:
                printf("nileswan/spi/flash: waking\n");
                sleeping = false;
                rxBuf.Pop(NULL, 1);
                txBuf.Push(NULL, 3);
                txBuf.Push(&spi_flash_dev_id, 1);
                break;
            case NILE_FLASH_CMD_WRDI:
                printf("nileswan/spi/flash: write disable\n");
                sr1 &= ~NILE_FLASH_SR1_WEL;
                rxBuf.Pop(NULL, 1);
                break;
            case NILE_FLASH_CMD_WREN:
                printf("nileswan/spi/flash: write enable\n");
                sr1 |= NILE_FLASH_SR1_WEL;
                rxBuf.Pop(NULL, 1);
                break;
            case NILE_FLASH_CMD_SLEEP:
                printf("nileswan/spi/flash: sleeping\n");
                sleeping = true;
                rxBuf.Pop(NULL, 1);
                break;
            default:
                printf("nileswan/spi/flash: unknown command %02X\n", rxBuf.data[0]);
                rxBuf.pos = 0;
                break;
        }
    }
    return rx;
}

void NileFlash::Serialize(Serializer& s)
{
	// TODO
}
