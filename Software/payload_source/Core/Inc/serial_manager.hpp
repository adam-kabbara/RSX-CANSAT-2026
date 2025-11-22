/*
 * serialManager.h
 *
 *  Created on: Oct 11, 2025
 *      Author: avaniyadav
 */

#ifndef INC_SERIAL_MANAGER_HPP_
#define INC_SERIAL_MANAGER_HPP_

#include "global_includes.hpp"

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

#ifdef __cplusplus
}
#endif

class SerialManager
{
private:
    UART_HandleTypeDef* serialPort;

public:
    SerialManager(UART_HandleTypeDef& port)
        : serialPort(&port)
        {}

    int get_data(char *cmd_buff);

    void sendErrorMsg(const char *msg);

    void sendInfoMsg(const char *msg);

    void sendErrorDataMsg(const char *format, ...);

    void sendInfoDataMsg(const char *format, ...);

    void sendTelemetry(char *buff);

    // void sendLogFile(FILE* log);

};


#endif /* INC_SERIAL_MANAGER_HPP_ */
