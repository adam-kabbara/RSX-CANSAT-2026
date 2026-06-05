#ifndef INC_RUNCAM_HPP
#define INC_RUNCAM_HPP

#include <stdint.h>
#include "global_includes.hpp"

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include "main.h"

#ifdef __cplusplus
}
#endif

class RunCam
{
private:
    GPIO_TypeDef* tx_port;
    uint16_t      tx_pin;
    GPIO_TypeDef* rx_port;
    uint16_t      rx_pin;
    bool          is_recording;

    // Private helpers for serialization and checksums
    uint8_t crc8_dvb_s2(uint8_t crc, unsigned char a);
    uint8_t compound_crc(uint8_t* data, int len);
    void    delay_cycles(uint32_t cycles);
    void    bitBangWriteByte(uint8_t byte);
    uint8_t bitBangReadByte();
    void    sendPacket(uint8_t* packet, int length);

public:
    RunCam();

    void Init(CameraID id);

    bool probeDevice();
    void startRecording();
    void stopRecording();
    bool getRecordingState() const { return is_recording; }
};

#endif /* INC_RUNCAM_HPP */
