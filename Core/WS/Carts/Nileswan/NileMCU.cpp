#include "pch.h"
#include "Utilities/Serializer.h"
#include "WS/Carts/Nileswan/NileMCU.h"
#include "WS/Carts/Nileswan/WsCartNileswan.h"
#include "WS/Carts/Nileswan/hardware.h"
#include "WS/Carts/WsRtc.h"
#include "WS/WsConsole.h"

NileMCU::NileMCU(WsCartNileswan *_parent)
	: parent(_parent), txBuf(1024), rxBuf(1024)
{
    memset(&persistent, 0, sizeof(persistent));
}

NileMCU::~NileMCU()
{
}

void NileMCU::Reset(bool _boot_mode)
{
    memset(&state, 0, sizeof(state));
    state.boot_mode = _boot_mode;
    state.cdc_unget = -1;

    txBuf.Reset();
    rxBuf.Reset();
}

void NileMCU::SendSpiResponse(uint16_t len, const void *buffer) {
    uint16_t key = len << 1;
    txBuf.Push((const uint8_t*) &key, 2);
    txBuf.Push((const uint8_t*) buffer, len);
}

void NileMCU::SendSpiBootAck(bool is_ack) {
    uint8_t r = 0;
    txBuf.Push(&r, 1); // empty byte
    r = is_ack ? NILE_MCU_BOOT_ACK : NILE_MCU_BOOT_NACK;
    txBuf.Push(&r, 1); // ACK
    state.boot_waiting_ack = true;
}

uint8_t NileMCU::SpiExchangeBoot(uint8_t tx) {
    uint8_t rx = NILE_MCU_BOOT_NACK;
    if (txBuf.pos && txBuf.Pop(&rx, 1))
        return rx;
    rxBuf.Push(&tx, 1);

    if (rxBuf.pos) {
        if ((state.boot_waiting_ack || !state.boot_step) && rxBuf.data[0] == NILE_MCU_BOOT_ACK) {
            printf("nileswan/spi/mcu/boot: ack\n");
            state.boot_waiting_ack = false;
            rxBuf.Pop(NULL, 1);
            if (state.boot_cmd == NILE_MCU_BOOT_JUMP && state.boot_step == 2) {
                printf("nileswan/spi/mcu/boot: stub: jump to %08X\n", state.boot_dest_address);
                state.boot_mode = false;
            }
        } else if (state.boot_waiting_ack) {
            rxBuf.Pop(NULL, 1);
            return rx;
        } else if (state.boot_step) {
            switch (state.boot_cmd) {
                case NILE_MCU_BOOT_JUMP:
                case NILE_MCU_BOOT_WRITE_MEMORY: {
                    if (state.boot_step == 1) {
                        if (rxBuf.pos < 5) return rx;
                        state.boot_dest_address = (rxBuf.data[0] << 24)
				| (rxBuf.data[1] << 16)
				| (rxBuf.data[2] << 8)
				| rxBuf.data[3];
                        state.boot_step = 2;
                        rxBuf.Pop(NULL, 5);
                        SendSpiBootAck(true);
                    }
                    if (state.boot_step == 2) {
                        if (rxBuf.pos < 1) return rx;
                        int len = rxBuf.data[0] + 1;
                        if (rxBuf.pos < len + 2) return rx;
                        printf("nileswan/spi/mcu/boot: stub: write %d bytes to %08X\n", len, state.boot_dest_address);
                        state.boot_step = 0;
                        rxBuf.Pop(NULL, len + 2);
                        SendSpiBootAck(true);
                    }
                } break;
                case NILE_MCU_BOOT_READ_MEMORY: {
                    if (state.boot_step == 1) {
                        if (rxBuf.pos < 5) return rx;
                        state.boot_dest_address = (rxBuf.data[0] << 24)
				| (rxBuf.data[1] << 16)
				| (rxBuf.data[2] << 8)
				| rxBuf.data[3];
                        state.boot_step = 2;
                        rxBuf.Pop(NULL, 5);
                        SendSpiBootAck(true);
                    }
                    if (state.boot_step == 2) {
                        if (rxBuf.pos < 2) return rx;
                        int len = rxBuf.data[0] + 1;
                        printf("nileswan/spi/mcu/boot: stub: read %d bytes from %08X\n", len, state.boot_dest_address);
                        state.boot_step = 0;
                        rxBuf.Pop(NULL, 2);
                        SendSpiBootAck(true);
                        // TODO: Send anything in response.
                    }
                } break;
                case NILE_MCU_BOOT_ERASE_MEMORY: {
                    if (state.boot_step == 1) {
                        if (rxBuf.pos < 3) return rx;
                        state.boot_erase_count = (rxBuf.data[0] << 8) | rxBuf.data[1];
                        state.boot_step = 2;
                        rxBuf.Pop(NULL, 3);
                        SendSpiBootAck(true);
                    }
                    if (state.boot_step == 2) {
                        if (rxBuf.pos < state.boot_erase_count*2+3) return rx;
                        printf("nileswan/spi/mcu/boot: stub: erase %d sectors\n", state.boot_erase_count);
                        state.boot_step = 0;
                        rxBuf.Pop(NULL, state.boot_erase_count*2+3);
                        SendSpiBootAck(true);
                    }
                } break;
            }
        } else if (rxBuf.data[0] == NILE_MCU_BOOT_START) {
            if (!state.boot_started) {
                printf("nileswan/spi/mcu/boot: start\n");
                SendSpiBootAck(true);
                state.boot_started = true;
                rxBuf.Pop(NULL, 1);
                return rx;
            }

            if (rxBuf.pos < 3) return rx;
            if ((rxBuf.data[1] ^ 0xFF) != rxBuf.data[2]) {
                printf("nileswan/spi/mcu/boot: command ID transfer error (%02X %02X)\n", rxBuf.data[1], rxBuf.data[2]);
                rxBuf.Pop(NULL, 3);
                return rx;
            }

            state.boot_cmd = rxBuf.data[1];
            switch (rxBuf.data[1]) {
                case NILE_MCU_BOOT_START: {
                    printf("nileswan/spi/mcu/boot: start\n");
                    SendSpiBootAck(true);
                } break;
                case NILE_MCU_BOOT_ERASE_MEMORY: {
                    printf("nileswan/spi/mcu/boot: erase memory\n");
                    state.boot_step = 1;
                    SendSpiBootAck(true);
                } break;
                case NILE_MCU_BOOT_WRITE_MEMORY: {
                    printf("nileswan/spi/mcu/boot: write memory\n");
                    state.boot_step = 1;
                    SendSpiBootAck(true);
                } break;
                case NILE_MCU_BOOT_READ_MEMORY: {
                    printf("nileswan/spi/mcu/boot: read memory\n");
                    state.boot_step = 1;
                    SendSpiBootAck(true);
                } break;
                case NILE_MCU_BOOT_JUMP: {
                    printf("nileswan/spi/mcu/boot: jump\n");
                    state.boot_step = 1;
                    SendSpiBootAck(true);
                } break;
                default: {
                    printf("nileswan/spi/mcu/boot: unknown command %02X\n", rxBuf.data[1]);
                    SendSpiBootAck(false);
                } break;
            }
            rxBuf.Pop(NULL, 3);
        } else {
            rxBuf.Pop(NULL, 1);
        }
    }

    return rx;
}

static const uint8_t rtc_cmd_rx_size[16] = {0, 0, 1, 0, 7, 0, 3, 0, 2, 0, 2, 0, 0, 0, 0, 0};
static const uint8_t rtc_cmd_tx_size[16] = {0, 0, 0, 1, 0, 7, 0, 3, 0, 2, 0, 2, 0, 0, 0, 0};

void NileMCU::RtcTransfer(uint8_t cmd, uint8_t* buf) {
    cmd = (cmd & 0xF) | 0x60;

    WsRtc *rtc = parent->GetRtc();
    int cmdLen = rtc->GetCommandLength(cmd);
    rtc->WriteBits(cmd, 8);

    if(cmd & 1) {
        for(int i = 0; i < cmdLen; i++) {
            buf[i] = rtc->ReadBits(8);
        }
    } else {
        for(int i = 0; i < cmdLen; i++) {
            rtc->WriteBits(buf[i], 8);
        }
    }
}

uint8_t NileMCU::SpiExchange(uint8_t tx) {
    if (state.boot_mode) {
        return SpiExchangeBoot(tx);
    }

    uint8_t rx = 0xFF;
    uint8_t response[512];
    if (txBuf.Pop(&rx, 1))
        return rx;
    rxBuf.Push(&tx, 1);

    if (rxBuf.pos) {
        // synchronize to command
        if (rxBuf.data[0] == 0xFF) {
            rxBuf.Pop(NULL, 1);
            return rx;
        }
        if (rxBuf.pos < 2) {
            return rx;
        }

        uint16_t cmd = rxBuf.data[0] & 0x7F;
        uint16_t arg = (rxBuf.data[0] >> 7) | rxBuf.data[1] << 1;
        switch (cmd) {
            case MCU_SPI_CMD_FREQ: {
                printf("nileswan/spi/mcu: set SPI speed to %d (no-op)\n", arg);
                rxBuf.Pop(NULL, 2);
                response[0] = 1;
                SendSpiResponse(1, response);
            } break;
            case MCU_SPI_CMD_ID: {
                printf("nileswan/spi/mcu: query MCU UID\n");
                rxBuf.Pop(NULL, 2);
                for (int i = 0; i < 12; i++)
                    response[i] = i;
                SendSpiResponse(12, response);
            } break;
            case MCU_SPI_CMD_VERSION: {
                printf("nileswan/spi/mcu: query MCU firmware version\n");
                rxBuf.Pop(NULL, 2);
                response[0] = NILE_EMULATED_MCU_MAJOR;
                response[1] = NILE_EMULATED_MCU_MAJOR >> 8;
                response[2] = NILE_EMULATED_MCU_MINOR;
                response[3] = NILE_EMULATED_MCU_MINOR >> 8;
                SendSpiResponse(4, response);
            } break;
            case MCU_SPI_CMD_EEPROM_MODE: {
                printf("nileswan/spi/mcu: set EEPROM mode to %d\n", arg);
                persistent.eeprom_mode = arg;
                rxBuf.Pop(NULL, 2);
                response[0] = 1;
                SendSpiResponse(1, response);
            } break;
            case MCU_SPI_CMD_EEPROM_READ: {
                if (arg == 0) arg = 512;
                if (rxBuf.pos < 4) break;
                uint16_t address = rxBuf.data[2] | (rxBuf.data[3] << 8);
                printf("nileswan/spi/mcu: read %d words from EEPROM address %04X\n", arg, address);
                rxBuf.Pop(NULL, 4);
                SendSpiResponse(2 * arg, persistent.eeprom_data + address);
            } break;
            case MCU_SPI_CMD_EEPROM_GET_MODE: {
                printf("nileswan/spi/mcu: get EEPROM mode (%d)\n", persistent.eeprom_mode);
                rxBuf.Pop(NULL, 2);
                response[0] = persistent.eeprom_mode;
                SendSpiResponse(1, response);
            } break;
            case MCU_SPI_CMD_SET_SAVE_ID: {
                if (rxBuf.pos < 6) break;
                persistent.save_id = rxBuf.data[2]
                    | (rxBuf.data[3] << 8)
                    | (rxBuf.data[4] << 16)
                    | (rxBuf.data[5] << 24);
                printf("nileswan/spi/mcu: set save ID to %d\n", persistent.save_id);
                rxBuf.Pop(NULL, 6);
                response[0] = 1;
                SendSpiResponse(1, response);
            } break;
            case MCU_SPI_CMD_GET_SAVE_ID: {
                printf("nileswan/spi/mcu: get save ID (%d)\n", persistent.save_id);
                rxBuf.Pop(NULL, 2);
                response[0] = persistent.save_id;
                response[1] = persistent.save_id >> 8;
                response[2] = persistent.save_id >> 16;
                response[3] = persistent.save_id >> 24;
                SendSpiResponse(4, response);
            } break;
            case MCU_SPI_CMD_USB_CDC_READ: {
                if (arg == 0) arg = 512;
                if (arg > NILE_MCU_MAX_PER_USB_CDC_PACKET) arg = NILE_MCU_MAX_PER_USB_CDC_PACKET;
                rxBuf.Pop(NULL, 2);
                uint16_t len = 0;
                for (; len < arg; len++) {
                    if (state.cdc_unget >= 0) {
                        response[len] = state.cdc_unget;
                        state.cdc_unget = -1;
                        continue;
                    }
                    // TODO
                    /* if (!Comm_RecvByte(response + len)) */
                        break;
                }
                printf("nileswan/spi/mcu: USB serial read %d bytes, found %d\n", arg, len);
                SendSpiResponse(len, response);
            } break;
            case MCU_SPI_CMD_USB_CDC_WRITE: {
                if (arg == 0) arg = 512;
                if (rxBuf.pos < 2+arg) break;
                rxBuf.Pop(NULL, 2);
                printf("nileswan/spi/mcu: USB serial write %d bytes\n", arg);
                int len = 0;
                // TODO
                for (; len < arg; len++) {
                    /* if (!Comm_SendByte(rxBuf.data[len])) */
                        break;
                }
                rxBuf.Pop(NULL, arg);
                SendSpiResponse(2, &len);
            } break;
            case MCU_SPI_CMD_USB_CDC_AVAILABLE: {
                // TODO: implement
                uint16_t len = state.cdc_unget < 0 ? 0 : 1;
                // uint8_t c;
                if (!len) {
                    // TODO
                    /* if (Comm_RecvByte(&c)) {
                        state.cdc_unget = c;
                        len = 1;
                    } */
                }
                printf("nileswan/spi/mcu: USB serial available = %d\n", len);
                rxBuf.Pop(NULL, 2);
                SendSpiResponse(2, &len);
            } break;
            case MCU_SPI_CMD_USB_CDC_FLUSH: {
                state.cdc_unget = -1;
                printf("nileswan/spi/mcu: USB serial flush\n");
                rxBuf.Pop(NULL, 2);
                SendSpiResponse(0, response);
            } break;
            case MCU_SPI_CMD_ECHO: {
                if (arg == 0) arg = 512;
                if (rxBuf.pos < 2+arg) break;
                rxBuf.Pop(NULL, 2);
                rxBuf.Pop(response, arg);
                SendSpiResponse(arg, response);
            } break;
            /* case MCU_SPI_CMD_WRITE_REG: {
                if (rxBuf.pos < 4) break;
                rxBuf.Pop(NULL, 2);
                rxBuf.Pop(response, arg);
                SendSpiResponse(arg, response);
            } break; */
            case MCU_SPI_CMD_RTC_COMMAND: {
                int rx_bytes = rtc_cmd_rx_size[arg & 0xF];
                int tx_bytes = rtc_cmd_tx_size[arg & 0xF];
                if (rxBuf.pos < 2+rx_bytes) break;
                rxBuf.Pop(NULL, 2);
                rxBuf.Pop(response, rx_bytes);
                RtcTransfer(arg & 0xF, response);
                SendSpiResponse(tx_bytes, response);
            } break;
            case MCU_SPI_CMD_ACCEL_POLL: {
                if (arg) {
                    printf("nileswan/spi/mcu: stub: enable accelerometer polling, %d Hz\n", arg);
                } else {
                    printf("nileswan/spi/mcu: stub: disable accelerometer polling\n");
                }
                response[0] = 1;
                rxBuf.Pop(NULL, 2);
                SendSpiResponse(1, response);
            } break;
            case MCU_SPI_CMD_ACCEL_READ: {
                printf("nileswan/spi/mcu: stub: read accelerometer position\n");
                response[0] = 0x00;
                response[1] = 0x00;
                response[2] = 0x00;
                response[3] = 0x04;
                response[4] = 0x00;
                response[5] = 0x00;
                rxBuf.Pop(NULL, 2);
                SendSpiResponse(6, response);
            } break;
            default: {
                printf("nileswan/spi/mcu: unknown command %02X %04X\n", cmd, arg);
                rxBuf.Pop(NULL, 2);
                // send 0x0001 for error
                response[0] = 0x01;
                response[1] = 0x00;
                txBuf.Push(response, 2);
            } break;
        }
    }

    return rx;
}

void NileMCU::Serialize(Serializer& s)
{
    SV(rxBuf);
    SV(txBuf);
    
    SV(state.boot_mode);
    SV(state.boot_started);
    SV(state.boot_waiting_ack);
    SV(state.boot_cmd);
    SV(state.boot_step);
    SV(state.boot_erase_count);
    SV(state.boot_dest_address);
    SV(state.cdc_unget);
    SV(persistent.eeprom_mode);
    SV(persistent.save_id);
    SVArray(persistent.eeprom_data, 1024);
}
