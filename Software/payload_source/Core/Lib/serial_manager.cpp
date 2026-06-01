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
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    this->sendErrorMsg(buffer);
}

void SerialManager::sendInfoDataMsg(const char *format, ...)
{
    char buffer[RESP_SIZE];

    va_list args;
    va_start(args, format);

    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    this->sendInfoMsg(buffer);
}

void SerialManager::sendTelemetry(char *buff)
{
	HAL_UART_Transmit(serialPort, (uint8_t*)buff, strlen(buff), HAL_MAX_DELAY);
}

void SerialManager::sendLogBegin()
{
	const char *beginMsg = "$LOGFILE:BEGIN\r";
	HAL_UART_Transmit(serialPort, (uint8_t*)beginMsg, strlen(beginMsg), HAL_MAX_DELAY);
}

void SerialManager::sendLogEnd()
{
	const char *endMsg = "$LOGFILE:END\r";
	HAL_UART_Transmit(serialPort, (uint8_t*)endMsg, strlen(endMsg), HAL_MAX_DELAY);
}

void SerialManager::sendLogLine(const char *line)
{
    HAL_UART_Transmit(serialPort, (uint8_t*)line, strlen(line), HAL_MAX_DELAY);
}

