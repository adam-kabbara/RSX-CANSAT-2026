/*  EEPROMsimple.h - Library for reading and writing data from an STM32 to a 25LC1024 chip.
 *  Ported from the Arduino version originally created by J. B. Gallaher (07/09/2016)
 *  and made into a library by David Dubins (November 12th, 2018).
 *  STM32 HAL port released into the public domain.
 *
 *  Usage:
 *    - Pass your SPI handle (e.g. &hspi1) and CS GPIO port/pin to the constructor.
 *    - Call SetMode() once before use (or rely on the constructor overload).
 *    - All address arguments are 24-bit (max 0x1FFFF for the 25LC1024).
 */

#ifndef EEPROM_SIMPLE_H
#define EEPROM_SIMPLE_H

#include "stm32g4xx_hal.h"   // Replace xxxx with your STM32 family, e.g. stm32f4xx_hal.h

/* ---------- 25LC1024 SPI opcodes ---------- */
#define EEPROM_WRITE   0x02
#define EEPROM_READ    0x03
#define EEPROM_WREN    0x06   // Write-enable latch
#define EEPROM_RDSR 0x05

/* ---------- Timing ---------- */
#define EEPROM_WRITE_DELAY_MS  10   // Max page-write cycle time per datasheet

/* ---------- SPI timeout (ms) ---------- */
#define EEPROM_SPI_TIMEOUT     100

#ifdef __cplusplus

class EEPROMsimple {
public:
    /* Constructor – supply SPI handle, CS port, and CS pin at construction time.
     * Example:  EEPROMsimple eeprom(&hspi1, GPIOA, GPIO_PIN_4); */
    EEPROMsimple(SPI_HandleTypeDef *hspi,
                 GPIO_TypeDef      *csPort,
                 uint16_t           csPin);

    ~EEPROMsimple();

    /* (Re-)configure CS pin and store SPI handle. Call once after construction
     * if you need to change the CS pin at runtime. */
    void SetMode(SPI_HandleTypeDef *hspi,
                 GPIO_TypeDef      *csPort,
                 uint16_t           csPin);

    /* ----- Byte ----- */
    void  WriteByte(uint32_t address, uint8_t data);
    uint8_t ReadByte(uint32_t address);

    void WriteByteArray(uint32_t address, uint8_t *data, uint16_t len);
    void ReadByteArray (uint32_t address, uint8_t *data, uint16_t len);

    /* ----- int (16-bit signed) ----- */
    void WriteInt(uint32_t address, int16_t data);
    int16_t ReadInt(uint32_t address);

    void WriteIntArray(uint32_t address, int16_t *data, uint16_t len);
    void ReadIntArray (uint32_t address, int16_t *data, uint16_t len);

    /* ----- unsigned int (16-bit unsigned) ----- */
    void WriteUnsignedInt(uint32_t address, uint16_t data);
    uint16_t ReadUnsignedInt(uint32_t address);

    void WriteUnsignedIntArray(uint32_t address, uint16_t *data, uint16_t len);
    void ReadUnsignedIntArray (uint32_t address, uint16_t *data, uint16_t len);

    /* ----- long (32-bit signed) ----- */
    void WriteLong(uint32_t address, int32_t data);
    int32_t ReadLong(uint32_t address);

    void WriteLongArray(uint32_t address, int32_t *data, uint16_t len);
    void ReadLongArray (uint32_t address, int32_t *data, uint16_t len);

    /* ----- unsigned long (32-bit unsigned) ----- */
    void WriteUnsignedLong(uint32_t address, uint32_t data);
    uint32_t ReadUnsignedLong(uint32_t address);

    void WriteUnsignedLongArray(uint32_t address, uint32_t *data, uint16_t len);
    void ReadUnsignedLongArray (uint32_t address, uint32_t *data, uint16_t len);

    /* ----- float (32-bit IEEE 754) ----- */
    void  WriteFloat(uint32_t address, float data);
    float ReadFloat(uint32_t address);

    void WriteFloatArray(uint32_t address, float *data, uint16_t len);
    void ReadFloatArray (uint32_t address, float *data, uint16_t len);

    uint8_t ReadStatus();

private:
    SPI_HandleTypeDef *_hspi;
    GPIO_TypeDef      *_csPort;
    uint16_t           _csPin;

    /* Low-level helpers */
    void     _csLow();
    void     _csHigh();
    void     _sendWREN();
    void     _sendAddress(uint32_t address);
    void     _writeRaw(uint32_t address, uint8_t *buf, uint16_t len);
    void     _readRaw (uint32_t address, uint8_t *buf, uint16_t len);
};

#endif /* __cplusplus */
#endif /* EEPROM_SIMPLE_H */