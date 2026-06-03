#include "runcam.hpp"
#include "serial_manager.hpp"

extern SerialManager serial;

RunCam::RunCam() : tx_port(nullptr), tx_pin(0), rx_port(nullptr), rx_pin(0), is_recording(false) {}

void RunCam::init(GPIO_TypeDef* tx_gpio_port, uint16_t tx_gpio_pin, 
                  GPIO_TypeDef* rx_gpio_port, uint16_t rx_gpio_pin)
{
    tx_port = tx_gpio_port;
    tx_pin  = tx_gpio_pin;
    rx_port = rx_gpio_port;
    rx_pin  = rx_gpio_pin;

    // Spin up Core Trace & DWT cycle counter clocks
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    *((volatile uint32_t*)0xE0001FB0) = 0xC5ACCE55;
    DWT->CTRL |= (1UL << 0);                        
    DWT->CYCCNT = 0;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    // Configure TX Pin as fast Push-Pull Output
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = tx_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;       
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; 
    HAL_GPIO_Init(tx_port, &GPIO_InitStruct);

    // Configure RX Pin as Input with internal Pull-up
    GPIO_InitStruct.Pin = rx_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;           
    GPIO_InitStruct.Pull = GPIO_PULLUP;               
    HAL_GPIO_Init(rx_port, &GPIO_InitStruct);

    // Clamp transmitter line to standard UART high idle state
    tx_port->BSRR = tx_pin;
    HAL_Delay(100);
}

inline void RunCam::delay_us(uint32_t us) 
{
    uint32_t startTick = DWT->CYCCNT;
    uint32_t targetTicks = us * 170; // Hardcoded to your 170MHz Clock Tree
    while ((DWT->CYCCNT - startTick) < targetTicks);
}

uint8_t RunCam::crc8_dvb_s2(uint8_t crc, unsigned char a)
{
    crc ^= a;
    for (int ii = 0; ii < 8; ++ii) {
        if (crc & 0x80) crc = (crc << 1) ^ 0xD5;
        else            crc = crc << 1;
    }
    return crc;
}

uint8_t RunCam::compound_crc(uint8_t* data, int len)
{
    uint8_t crc = 0;
    for(int i = 0; i < len; i++) {
        crc = crc8_dvb_s2(crc, data[i]);
    }
    return crc;
}

void RunCam::bitBangWriteByte(uint8_t byte) 
{
    // START BIT (Drop Low)
    tx_port->BSRR = (tx_pin << 16); 
    delay_us(9);
    __NOP(); __NOP(); __NOP();

    // 8 DATA BITS (LSB first)
    for (int i = 0; i < 8; i++) 
    {
        if (byte & (1 << i)) tx_port->BSRR = tx_pin;          // HIGH
        else                 tx_port->BSRR = (tx_pin << 16);  // LOW
        delay_us(9);
        __NOP(); __NOP(); __NOP();
    }

    // STOP BIT (Return HIGH)
    tx_port->BSRR = tx_pin; 
    delay_us(9);
}

uint8_t RunCam::bitBangReadByte() 
{
    __disable_irq();
    uint8_t byte = 0;
    uint32_t timeout = 80000; 

    while ((rx_port->IDR & rx_pin) != 0)
    {
        if (--timeout == 0)
        {
            serial.sendErrorMsg("[RunCam] bitBangReadByte: timeout — RX line never went LOW (no start bit)");
            return 0x00;
        }
    }

    delay_us(4); 
    __NOP(); __NOP();

    for (int i = 0; i < 8; i++) 
    {
        delay_us(9); 
        __NOP(); __NOP(); __NOP();
        if ((rx_port->IDR & rx_pin) != 0) byte |= (1 << i); 
    }
    delay_us(9);
    __disable_irq();
    return byte;
}

void RunCam::sendPacket(uint8_t* packet, int length)
{
    __disable_irq(); 
    for (int i = 0; i < length; i++) {
        bitBangWriteByte(packet[i]);
    }
    __enable_irq();
}

bool RunCam::probeDevice()
{
    uint8_t packet[3] = {0xCC, 0x00, 0x00};
    packet[2] = compound_crc(packet, 2);

    serial.sendInfoDataMsg("[RunCam] probeDevice: RX idle state=%d (expect 1)", (rx_port->IDR & rx_pin) ? 1 : 0);
    serial.sendInfoDataMsg("[RunCam] probeDevice: sending handshake {0x%02X, 0x%02X, 0x%02X}", packet[0], packet[1], packet[2]);
    sendPacket(packet, 3);

    uint8_t sync_byte = bitBangReadByte();
    serial.sendInfoDataMsg("[RunCam] probeDevice: got response byte 0x%02X (expect 0xCC)", sync_byte);
    return (sync_byte == 0xCC);
}

void RunCam::toggleRecording()
{
    uint8_t packet[4] = {0xCC, 0x01, 0x01, 0x00};
    packet[3] = compound_crc(packet, 3);

    serial.sendInfoDataMsg("[RunCam] toggleRecording: sending {0x%02X, 0x%02X, 0x%02X, 0x%02X}", packet[0], packet[1], packet[2], packet[3]);
    sendPacket(packet, 4);
    is_recording = !is_recording;
    serial.sendInfoDataMsg("[RunCam] toggleRecording: done, is_recording=%d", (int)is_recording);
}