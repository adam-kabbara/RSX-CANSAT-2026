/*  EEPROMsimple.cpp - Library for reading and writing data from an STM32 to a 25LC1024 chip.
 *  Ported from the Arduino version originally created by J. B. Gallaher (07/09/2016)
 *  and made into a library by David Dubins (November 12th, 2018).
 *  STM32 HAL port released into the public domain.
 *
 *  Notes on the STM32 port
 *  ───────────────────────
 *  • Arduino's SPI.transfer() is replaced with HAL_SPI_Transmit / HAL_SPI_Receive /
 *    HAL_SPI_TransmitReceive from the STM32 HAL.
 *  • CS (chip-select) is driven manually via HAL_GPIO_WritePin, exactly as the
 *    original library drove it with digitalWrite().
 *  • The 10 ms write delay uses HAL_Delay (1 ms resolution) – identical behaviour
 *    to Arduino's delay().
 *  • Variable-length arrays (VLAs) used in the original are replaced with fixed
 *    stack buffers or heap allocation to stay compatible with MSVC/IAR/Keil.
 *    A compile-time guard caps single-call transfers at EEPROM_MAX_BUF_BYTES.
 *  • int / unsigned int / long / unsigned long are replaced with explicit-width
 *    stdint types (int16_t, uint16_t, int32_t, uint32_t) for portability.
 *
 *  Configure your SPI peripheral (CubeMX or manually) as:
 *    Mode          : Full-Duplex Master
 *    Data Size     : 8 bits
 *    First Bit     : MSB
 *    CPOL / CPHA   : Low / 1st Edge  (SPI Mode 0)
 *    Baud rate     : ≤ 20 MHz  (25LC1024 maximum)
 *    NSS           : Software (we toggle CS ourselves)
 */

#include "eeprom.hpp"
#include <string.h>   // memcpy

/* Maximum bytes transferred in one internal buffer (stack guard).
 * Increase if you need larger single-call array transfers. */
#define EEPROM_MAX_BUF_BYTES  512

/* ═══════════════════════════════════════════════════════
 *  Constructor / Destructor / SetMode
 * ═══════════════════════════════════════════════════════ */

EEPROMsimple::EEPROMsimple(SPI_HandleTypeDef *hspi,
                            GPIO_TypeDef      *csPort,
                            uint16_t           csPin)
{
    SetMode(hspi, csPort, csPin);
}

EEPROMsimple::~EEPROMsimple() { /* nothing to destruct */ }

void EEPROMsimple::SetMode(SPI_HandleTypeDef *hspi,
                            GPIO_TypeDef      *csPort,
                            uint16_t           csPin)
{
    _hspi   = hspi;
    _csPort = csPort;
    _csPin  = csPin;

    /* Ensure CS starts de-asserted (HIGH) */
    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);
}

/* ═══════════════════════════════════════════════════════
 *  Private helpers
 * ═══════════════════════════════════════════════════════ */

inline void EEPROMsimple::_csLow()
{
    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_RESET);
}

inline void EEPROMsimple::_csHigh()
{
    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);
}

/* Send Write-Enable (WREN) opcode – must precede every WRITE command */
void EEPROMsimple::_sendWREN()
{
    uint8_t cmd = EEPROM_WREN;
    _csLow();
    HAL_SPI_Transmit(_hspi, &cmd, 1, EEPROM_SPI_TIMEOUT);
    _csHigh();
}

/* Clock out the 3-byte (24-bit) address after a READ or WRITE opcode */
void EEPROMsimple::_sendAddress(uint32_t address)
{
    uint8_t addr[3] = {
        (uint8_t)(address >> 16),
        (uint8_t)(address >>  8),
        (uint8_t)(address)
    };
    HAL_SPI_Transmit(_hspi, addr, 3, EEPROM_SPI_TIMEOUT);
}

/* Low-level write: WREN → WRITE opcode → address → data → CS high → delay */
void EEPROMsimple::_writeRaw(uint32_t address, uint8_t *buf, uint16_t len)
{
    uint16_t totalLen = 4 + len;
    if (totalLen > EEPROM_MAX_BUF_BYTES) return;

    uint8_t txBuf[EEPROM_MAX_BUF_BYTES] = {0};
    uint8_t rxBuf[EEPROM_MAX_BUF_BYTES] = {0}; // Dummy storage for clean full-duplex clearing

    _sendWREN();

    // Pack header
    txBuf[0] = EEPROM_WRITE;
    txBuf[1] = (uint8_t)(address >> 16);
    txBuf[2] = (uint8_t)(address >> 8);
    txBuf[3] = (uint8_t)(address);

    // Append our payload data right into the sequential transmit packet
    memcpy(&txBuf[4], buf, len);

    _csLow();
    HAL_SPI_TransmitReceive(_hspi, txBuf, rxBuf, totalLen, EEPROM_SPI_TIMEOUT);
    _csHigh();

    HAL_Delay(EEPROM_WRITE_DELAY_MS);
}

/* Low-level read: READ opcode → address → receive data */
void EEPROMsimple::_readRaw(uint32_t address, uint8_t *buf, uint16_t len)
{
    // Frame buffer size = 1 byte (Command) + 3 bytes (Address) + len (Data Payload)
    uint16_t totalLen = 4 + len;
    
    // Safety check to avoid overrunning our temporary heap/stack arrays
    if (totalLen > EEPROM_MAX_BUF_BYTES) return;

    uint8_t txBuf[EEPROM_MAX_BUF_BYTES] = {0};
    uint8_t rxBuf[EEPROM_MAX_BUF_BYTES] = {0};

    // Pack the header
    txBuf[0] = EEPROM_READ;
    txBuf[1] = (uint8_t)(address >> 16);
    txBuf[2] = (uint8_t)(address >> 8);
    txBuf[3] = (uint8_t)(address);

    _csLow();
    // Swap the entire transaction array at once over the bus
    if (HAL_SPI_TransmitReceive(_hspi, txBuf, rxBuf, totalLen, EEPROM_SPI_TIMEOUT) == HAL_OK) {
        // Copy the captured shifted data out, offset past the 4-byte header space
        memcpy(buf, &rxBuf[4], len);
    }
    _csHigh();
}

/* ═══════════════════════════════════════════════════════
 *  Byte
 * ═══════════════════════════════════════════════════════ */

void EEPROMsimple::WriteByte(uint32_t address, uint8_t data)
{
    _writeRaw(address, &data, 1);
}

uint8_t EEPROMsimple::ReadByte(uint32_t address)
{
    uint8_t data = 0;
    _readRaw(address, &data, 1);
    return data;
}

void EEPROMsimple::WriteByteArray(uint32_t address, uint8_t *data, uint16_t len)
{
    _writeRaw(address, data, len);
}

void EEPROMsimple::ReadByteArray(uint32_t address, uint8_t *data, uint16_t len)
{
    _readRaw(address, data, len);
}

/* ═══════════════════════════════════════════════════════
 *  int16_t  (signed 16-bit, big-endian on the wire)
 * ═══════════════════════════════════════════════════════ */

void EEPROMsimple::WriteInt(uint32_t address, int16_t data)
{
    uint8_t temp[2] = {
        (uint8_t)(data >> 8),   // high byte
        (uint8_t)(data)         // low byte
    };
    _writeRaw(address, temp, 2);
}

int16_t EEPROMsimple::ReadInt(uint32_t address)
{
    uint8_t temp[2] = {0};
    _readRaw(address, temp, 2);
    return (int16_t)(((uint16_t)temp[0] << 8) | temp[1]);
}

void EEPROMsimple::WriteIntArray(uint32_t address, int16_t *data, uint16_t len)
{
    uint8_t temp[EEPROM_MAX_BUF_BYTES];
    uint16_t byteLen = len * 2;
    if (byteLen > EEPROM_MAX_BUF_BYTES) byteLen = EEPROM_MAX_BUF_BYTES; // safety clamp

    for (uint16_t i = 0, j = 0; i < len && j + 1 < EEPROM_MAX_BUF_BYTES; i++, j += 2) {
        temp[j]     = (uint8_t)(data[i] >> 8);
        temp[j + 1] = (uint8_t)(data[i]);
    }
    _writeRaw(address, temp, byteLen);
}

void EEPROMsimple::ReadIntArray(uint32_t address, int16_t *data, uint16_t len)
{
    uint8_t temp[EEPROM_MAX_BUF_BYTES];
    uint16_t byteLen = len * 2;
    if (byteLen > EEPROM_MAX_BUF_BYTES) byteLen = EEPROM_MAX_BUF_BYTES;

    _readRaw(address, temp, byteLen);
    for (uint16_t i = 0, j = 0; i < len; i++, j += 2) {
        data[i] = (int16_t)(((uint16_t)temp[j] << 8) | temp[j + 1]);
    }
}

/* ═══════════════════════════════════════════════════════
 *  uint16_t  (unsigned 16-bit)
 * ═══════════════════════════════════════════════════════ */

void EEPROMsimple::WriteUnsignedInt(uint32_t address, uint16_t data)
{
    uint8_t temp[2] = {
        (uint8_t)(data >> 8),
        (uint8_t)(data)
    };
    _writeRaw(address, temp, 2);
}

uint16_t EEPROMsimple::ReadUnsignedInt(uint32_t address)
{
    uint8_t temp[2] = {0};
    _readRaw(address, temp, 2);
    return (uint16_t)(((uint16_t)temp[0] << 8) | temp[1]);
}

void EEPROMsimple::WriteUnsignedIntArray(uint32_t address, uint16_t *data, uint16_t len)
{
    uint8_t temp[EEPROM_MAX_BUF_BYTES];
    uint16_t byteLen = len * 2;
    if (byteLen > EEPROM_MAX_BUF_BYTES) byteLen = EEPROM_MAX_BUF_BYTES;

    for (uint16_t i = 0, j = 0; i < len && j + 1 < EEPROM_MAX_BUF_BYTES; i++, j += 2) {
        temp[j]     = (uint8_t)(data[i] >> 8);
        temp[j + 1] = (uint8_t)(data[i]);
    }
    _writeRaw(address, temp, byteLen);
}

void EEPROMsimple::ReadUnsignedIntArray(uint32_t address, uint16_t *data, uint16_t len)
{
    uint8_t temp[EEPROM_MAX_BUF_BYTES];
    uint16_t byteLen = len * 2;
    if (byteLen > EEPROM_MAX_BUF_BYTES) byteLen = EEPROM_MAX_BUF_BYTES;

    _readRaw(address, temp, byteLen);
    for (uint16_t i = 0, j = 0; i < len; i++, j += 2) {
        data[i] = (uint16_t)(((uint16_t)temp[j] << 8) | temp[j + 1]);
    }
}

/* ═══════════════════════════════════════════════════════
 *  int32_t  (signed 32-bit, big-endian on the wire)
 * ═══════════════════════════════════════════════════════ */

void EEPROMsimple::WriteLong(uint32_t address, int32_t data)
{
    uint8_t temp[4] = {
        (uint8_t)(data >> 24),
        (uint8_t)(data >> 16),
        (uint8_t)(data >>  8),
        (uint8_t)(data)
    };
    _writeRaw(address, temp, 4);
}

int32_t EEPROMsimple::ReadLong(uint32_t address)
{
    uint8_t temp[4] = {0};
    _readRaw(address, temp, 4);
    return (int32_t)(((uint32_t)temp[0] << 24) |
                     ((uint32_t)temp[1] << 16) |
                     ((uint32_t)temp[2] <<  8) |
                      (uint32_t)temp[3]);
}

void EEPROMsimple::WriteLongArray(uint32_t address, int32_t *data, uint16_t len)
{
    uint8_t temp[EEPROM_MAX_BUF_BYTES];
    uint16_t byteLen = len * 4;
    if (byteLen > EEPROM_MAX_BUF_BYTES) byteLen = EEPROM_MAX_BUF_BYTES;

    for (uint16_t i = 0, j = 0; i < len && j + 3 < EEPROM_MAX_BUF_BYTES; i++, j += 4) {
        temp[j]     = (uint8_t)(data[i] >> 24);
        temp[j + 1] = (uint8_t)(data[i] >> 16);
        temp[j + 2] = (uint8_t)(data[i] >>  8);
        temp[j + 3] = (uint8_t)(data[i]);
    }
    _writeRaw(address, temp, byteLen);
}

void EEPROMsimple::ReadLongArray(uint32_t address, int32_t *data, uint16_t len)
{
    uint8_t temp[EEPROM_MAX_BUF_BYTES];
    uint16_t byteLen = len * 4;
    if (byteLen > EEPROM_MAX_BUF_BYTES) byteLen = EEPROM_MAX_BUF_BYTES;

    _readRaw(address, temp, byteLen);
    for (uint16_t i = 0, j = 0; i < len; i++, j += 4) {
        data[i] = (int32_t)(((uint32_t)temp[j]     << 24) |
                             ((uint32_t)temp[j + 1] << 16) |
                             ((uint32_t)temp[j + 2] <<  8) |
                              (uint32_t)temp[j + 3]);
    }
}

/* ═══════════════════════════════════════════════════════
 *  uint32_t  (unsigned 32-bit)
 * ═══════════════════════════════════════════════════════ */

void EEPROMsimple::WriteUnsignedLong(uint32_t address, uint32_t data)
{
    uint8_t temp[4] = {
        (uint8_t)(data >> 24),
        (uint8_t)(data >> 16),
        (uint8_t)(data >>  8),
        (uint8_t)(data)
    };
    _writeRaw(address, temp, 4);
}

uint32_t EEPROMsimple::ReadUnsignedLong(uint32_t address)
{
    uint8_t temp[4] = {0};
    _readRaw(address, temp, 4);
    return ((uint32_t)temp[0] << 24) |
           ((uint32_t)temp[1] << 16) |
           ((uint32_t)temp[2] <<  8) |
            (uint32_t)temp[3];
}

void EEPROMsimple::WriteUnsignedLongArray(uint32_t address, uint32_t *data, uint16_t len)
{
    uint8_t temp[EEPROM_MAX_BUF_BYTES];
    uint16_t byteLen = len * 4;
    if (byteLen > EEPROM_MAX_BUF_BYTES) byteLen = EEPROM_MAX_BUF_BYTES;

    for (uint16_t i = 0, j = 0; i < len && j + 3 < EEPROM_MAX_BUF_BYTES; i++, j += 4) {
        temp[j]     = (uint8_t)(data[i] >> 24);
        temp[j + 1] = (uint8_t)(data[i] >> 16);
        temp[j + 2] = (uint8_t)(data[i] >>  8);
        temp[j + 3] = (uint8_t)(data[i]);
    }
    _writeRaw(address, temp, byteLen);
}

void EEPROMsimple::ReadUnsignedLongArray(uint32_t address, uint32_t *data, uint16_t len)
{
    uint8_t temp[EEPROM_MAX_BUF_BYTES];
    uint16_t byteLen = len * 4;
    if (byteLen > EEPROM_MAX_BUF_BYTES) byteLen = EEPROM_MAX_BUF_BYTES;

    _readRaw(address, temp, byteLen);
    for (uint16_t i = 0, j = 0; i < len; i++, j += 4) {
        data[i] = ((uint32_t)temp[j]     << 24) |
                  ((uint32_t)temp[j + 1] << 16) |
                  ((uint32_t)temp[j + 2] <<  8) |
                   (uint32_t)temp[j + 3];
    }
}

/* ═══════════════════════════════════════════════════════
 *  float  (32-bit IEEE 754, stored as raw bytes)
 * ═══════════════════════════════════════════════════════ */

void EEPROMsimple::WriteFloat(uint32_t address, float data)
{
    uint8_t temp[4];
    memcpy(temp, &data, 4);   // portable – avoids strict-aliasing UB
    _writeRaw(address, temp, 4);
}

float EEPROMsimple::ReadFloat(uint32_t address)
{
    uint8_t temp[4] = {0};
    float data = 0.0f;
    _readRaw(address, temp, 4);
    memcpy(&data, temp, 4);   // portable reassembly
    return data;
}

void EEPROMsimple::WriteFloatArray(uint32_t address, float *data, uint16_t len)
{
    uint8_t temp[EEPROM_MAX_BUF_BYTES];
    uint16_t byteLen = len * 4;
    if (byteLen > EEPROM_MAX_BUF_BYTES) byteLen = EEPROM_MAX_BUF_BYTES;

    for (uint16_t i = 0, j = 0; i < len && j + 3 < EEPROM_MAX_BUF_BYTES; i++, j += 4) {
        memcpy(&temp[j], &data[i], 4);
    }
    _writeRaw(address, temp, byteLen);
}

void EEPROMsimple::ReadFloatArray(uint32_t address, float *data, uint16_t len)
{
    uint8_t temp[EEPROM_MAX_BUF_BYTES];
    uint16_t byteLen = len * 4;
    if (byteLen > EEPROM_MAX_BUF_BYTES) byteLen = EEPROM_MAX_BUF_BYTES;

    _readRaw(address, temp, byteLen);
    for (uint16_t i = 0, j = 0; i < len; i++, j += 4) {
        memcpy(&data[i], &temp[j], 4);
    }
}


uint8_t EEPROMsimple::ReadStatus()
{
    uint8_t tx[2] = { EEPROM_RDSR, 0x00 }; // Read Status Register Opcode + Dummy Byte
    uint8_t rx[2] = { 0x00, 0x00 };

    _csLow();
    HAL_SPI_TransmitReceive(_hspi, tx, rx, 2, EEPROM_SPI_TIMEOUT);
    _csHigh();

    return rx[1]; // The second byte contains the shifted status value from the EEPROM
}