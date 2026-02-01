/*
 * serial_manager.cpp
 *
 *  Manages all serial functions
 */

#include "serial_manager.hpp"

void SerialManager::sendErrorMsg(const char* msg)
{
    char buffer[RESP_SIZE];
    int len = snprintf(buffer, sizeof(buffer), "$E:%s\r", msg);

    size_t to_send = (len < (int)sizeof(buffer)) ? (size_t)len : (sizeof(buffer) - 1);
    HAL_UART_Transmit(serialPort, (uint8_t*)buffer, (uint16_t)to_send, HAL_MAX_DELAY);
}

void SerialManager::sendInfoMsg(const char* msg)
{
    char buffer[RESP_SIZE];
    int len = snprintf(buffer, sizeof(buffer), "$I:%s\r", msg);

    size_t to_send = (len < (int)sizeof(buffer)) ? (size_t)len : (sizeof(buffer) - 1);

    HAL_UART_Transmit(serialPort, (uint8_t*)buffer, (uint16_t)to_send, HAL_MAX_DELAY);
}


void SerialManager::sendErrorDataMsg(const char *format, ...)
{
	char buffer[RESP_SIZE];

    va_list args;
    va_start(args, format);
    int ret = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    this->sendErrorMsg(buffer);
}

void SerialManager::sendInfoDataMsg(const char *format, ...)
{
    char buffer[RESP_SIZE];

    va_list args;
    va_start(args, format);

    int ret = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    this->sendInfoMsg(buffer);
}

void SerialManager::sendTelemetry(char *buff)
{
	HAL_UART_Transmit(serialPort, (uint8_t*)buff, strlen(buff), HAL_MAX_DELAY);
}

void SerialManager::sendLogFile()
{
	const char *beginMsg = "$LOGFILE:BEGIN\r\n";
	const char *endMsg = "$LOGFILE:END\r\n";

    HAL_UART_Transmit(serialPort, (uint8_t*)beginMsg, strlen(beginMsg), HAL_MAX_DELAY);
    HAL_Delay(500);

    /* TODO: Update with EEPROM code
    while (fgets(line_buff, sizeof(line_buff), log))
    {
    	size_t len = strlen(line_buff);
    	if (len > 0 && (line_buff[len-1] == '\n' || line_buff[len-1] == '\r')) {
    		line_buff[len-1] = '\0';
    	}
        HAL_UART_Transmit(serialPort, (uint8_t*)line_buff, strlen(line_buff), HAL_MAX_DELAY);
        HAL_UART_Transmit(serialPort, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY); // println
        HAL_Delay(500);
    }
    */

    HAL_UART_Transmit(serialPort, (uint8_t*)endMsg, strlen(endMsg), HAL_MAX_DELAY);
}

