/*
 * serialManager.cpp
 *
 *  	Created on: Oct 25, 2025
 *      Author: avaniyadav
 *      @brief          :
 */

#include <serial_manager.h>
#include "global_includes.h"

void SerialManager::begin() // done (?)
{
	// Skip initialization, already initialized in main.c --> static void MX_USART1_UART_Init(void);
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
	    // handle error
	}

	HAL_Delay(1000);

}

int SerialManager::get_data(char* cmd_buff) // done
{
	memset(cmd_buff, 0, CMD_BUFF_SIZE);
	uint8_t ch;
	size_t idx = 0;
	const uint32_t per_byte_timeout_ms = 100; // edit timeout value (maybe add as macro)

	while (idx < (CMD_BUFF_SIZE - 1))
	{
	    if (HAL_UART_Receive(&huart1, &ch, 1, per_byte_timeout_ms) == HAL_OK)
	    {
	        if (ch == '\r') continue; // ignore CR if sender uses CRLF
	        if (ch == '\n') break;    // end of line
	        cmd_buff[idx++] = static_cast<char>(ch);
	    }
	    else
	    {
	        // timed out waiting for next byte -> stop reading
	        break;
	    }
	}

	cmd_buff[idx] = '\0';
	if (idx == 0){
		return 0;
	}
	return 1;
}

void SerialManager::sendErrorMsg(const char* msg) // done
{
    char buffer[RESP_SIZE];
    int len = snprintf(buffer, sizeof(buffer), "$E MSG:%s\r\n", msg);
    if (len <= 0) return;

    size_t to_send = (len < (int)sizeof(buffer)) ? (size_t)len : (sizeof(buffer) - 1);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, (uint16_t)to_send, HAL_MAX_DELAY);
}

void SerialManager::sendInfoMsg(const char* msg) // done (need to add error handling)
{
    char buffer[RESP_SIZE];
    int len = snprintf(buffer, sizeof(buffer), "$I MSG:%s", msg);
    if (len < 0){
    	// handle formatting error!
    	return;
    }

    size_t to_send = (len < (int)sizeof(buffer)) ? (size_t)len : (sizeof(buffer) - 1);

    // blocking transmit:
    if (HAL_UART_Transmit(&huart1, (uint8_t*)buffer, (uint16_t)to_send, HAL_MAX_DELAY) != HAL_OK){
    	// toggle some LED to indicate error
    }
}


void SerialManager::sendErrorDataMsg(const char *format, ...) // done
{
	char buffer[RESP_SIZE];

    va_list args;
    va_start(args, format);
    int ret = vsnprintf(buffer, sizeof(buffer), format, args);

    if (ret < 0) {
        // formatting error
        this->sendErrorMsg("<format error>");
        return;
    }

    // If truncated, ret >= sizeof(buffer). Buffer is still NUL-terminated.
    if ((size_t)ret >= sizeof(buffer)) {
        buffer[sizeof(buffer)-1] = '\0';
    }

    this->sendErrorMsg(buffer);
    va_end(args);
}

void SerialManager::sendInfoDataMsg(const char *format, ...) // done
{

    char buffer[RESP_SIZE];

    va_list args;
    va_start(args, format);

    int ret = vsnprintf(buffer, sizeof(buffer), format, args);

    if (ret < 0) {
		// formatting error
		this->sendErrorMsg("<format error>");
		return;
    }

	// If truncated, ret >= sizeof(buffer). Buffer is still NUL-terminated.
	if ((size_t)ret >= sizeof(buffer)) {
		buffer[sizeof(buffer)-1] = '\0';
	}

    this->sendInfoMsg(buffer);

    va_end(args);
}

void SerialManager::sendTelemetry(char *buff)
{
	HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), HAL_MAX_DELAY);
}

void SerialManager::sendLogFile(File log)
{

	const char *beginMsg = "$LOGFILE:BEGIN\r\n";
	const char *endMsg = "$LOGFILE:END\r\n";

    HAL_UART_Transmit(&huart1, (uint8_t*)beginMsg, strlen(beginMsg), HAL_MAX_DELAY);
    HAL_Delay(500);

    while (fgets(lineBuff, sizeof(lineBuff), log))
    {
    	size_t len = strlen(lineBuff);
    	if (len > 0 && (lineBuff[len-1] == '\n' || lineBuff[len-1] == '\r')) {
    		lineBuff[len-1] = '\0';
    	}
        HAL_UART_Transmit(&huart1, (uint8_t*)lineBuff, strlen(lineBuff), HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY); // println
        HAL_Delay(500);
    }

    HAL_UART_Transmit(&huart1, (uint8_t*)endMsg, strlen(endMsg), HAL_MAX_DELAY);
}
