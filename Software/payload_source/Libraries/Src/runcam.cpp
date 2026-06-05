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

    // Spin up DWT cycle counter
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    *((volatile uint32_t*)0xE0001FB0) = 0xC5ACCE55; // unlock DWT LAR
    DWT->CTRL |= (1UL << 0);
    DWT->CYCCNT = 0;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // TX pin — fast push-pull output, start HIGH (UART idle)
    GPIO_InitStruct.Pin   = tx_pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(tx_port, &GPIO_InitStruct);
    tx_port->BSRR = tx_pin; // idle HIGH

    // RX pin — input with pull-up
    GPIO_InitStruct.Pin  = rx_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(rx_port, &GPIO_InitStruct);

    HAL_Delay(100);
}

// 170 MHz / 115200 = 1476 cycles per bit
inline void RunCam::delay_cycles(uint32_t cycles)
{
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles);
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
    for (int i = 0; i < len; i++)
        crc = crc8_dvb_s2(crc, data[i]);
    return crc;
}

void RunCam::bitBangWriteByte(uint8_t byte)
{
    // start bit (LOW)
    tx_port->BSRR = ((uint32_t)tx_pin << 16);
    delay_cycles(1476);

    // 8 data bits LSB first
    for (int i = 0; i < 8; i++) {
        if (byte & (1 << i)) {
            tx_port->BSRR = tx_pin; // HIGH
        } else {
            tx_port->BSRR = ((uint32_t)tx_pin << 16); // LOW
        }
        delay_cycles(1476);
    }

    // stop bit (HIGH)
    tx_port->BSRR = tx_pin;
    delay_cycles(1476);
}

uint8_t RunCam::bitBangReadByte()
{
    uint8_t byte = 0;

    // 1. Wait for start bit (RX drops LOW) with interrupts ENABLED
    uint32_t start = DWT->CYCCNT;
    while ((rx_port->IDR & rx_pin) != 0) {
        if ((DWT->CYCCNT - start) > (170000UL * 100)) { // 100ms timeout
            return 0x00;
        }
    }

    // 2. Start bit detected! Immediately mask interrupts to lock timing
    __disable_irq();

    // Step to the middle of the start bit, then precisely jump bit by bit
    uint32_t target_cycle = DWT->CYCCNT + 738; 

    // 8 data bits
    for (int i = 0; i < 8; i++) {
        target_cycle += 1476;
        while ((int32_t)(target_cycle - DWT->CYCCNT) > 0); 

        if ((rx_port->IDR & rx_pin) != 0) {
            byte |= (1 << i);
        }
    }

    // Wait for stop bit window to clear out
    target_cycle += 1476;
    while ((int32_t)(target_cycle - DWT->CYCCNT) > 0);

    // 3. Release interrupts back to the system
    __enable_irq();

    return byte;
}

void RunCam::sendPacket(uint8_t* packet, int length)
{
    __disable_irq();
    for (int i = 0; i < length; i++)
        bitBangWriteByte(packet[i]);
    __enable_irq();
}

bool RunCam::probeDevice()
{
    uint8_t packet[3] = {0xCC, 0x00, 0x00};
    packet[2] = compound_crc(packet, 2);

    // sendPacket handles its own brief __disable_irq() block internally
    sendPacket(packet, 3);
    
    // Read the response byte with interrupts active during the idle wait
    uint8_t sync_byte = bitBangReadByte();

    // Expecting 0xCC back from the camera
    return (sync_byte == 0xCC);
}

void RunCam::toggleRecording()
{
    uint8_t packet[4] = {0xCC, 0x01, 0x01, 0x00};
    packet[3] = compound_crc(packet, 3);

    serial.sendInfoDataMsg("[RunCam] toggleRecording: sending {0x%02X, 0x%02X, 0x%02X, 0x%02X}",
                           packet[0], packet[1], packet[2], packet[3]);
    sendPacket(packet, 4);
    is_recording = true;
    //serial.sendInfoDataMsg("[RunCam] toggleRecording: done, is_recording=%d", (int)is_recording);
}

void RunCam::stopRecording()
{
	uint8_t packet[4] = {0xCC, 0x01, 0x04, 0x00};
	packet[3] = compound_crc(packet, 3);

	//serial.sendInfoDataMsg("[RunCam] toggleRecording: sending {0x%02X, 0x%02X, 0x%02X, 0x%02X}",
						   //packet[0], packet[1], packet[2], packet[3]);
	sendPacket(packet, 4);
	is_recording = false;
	//serial.sendInfoDataMsg("[RunCam] toggleRecording: done, is_recording=%d", (int)is_recording);
}