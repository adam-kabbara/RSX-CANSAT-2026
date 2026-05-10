/*
 * serial_manager.cpp
 *
 *  Manages all serial functions
 */

#include "serial_manager.hpp"

int SerialManager::get_data(char* cmd_buff)
{
	memset(cmd_buff, 0, CMD_BUFF_SIZE);
	uint8_t ch;
	size_t idx = 0;

	while (idx < (CMD_BUFF_SIZE - 1))
	{
	    if (HAL_UART_Receive(serialPort, &ch, 1, 100) == HAL_OK)
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
	if (idx == 0)
	{
		return 0;
	}

	return 1;
}

void SerialManager::sendErrorMsg(const char* msg)
{
    char buffer[RESP_SIZE];
    int len = snprintf(buffer, sizeof(buffer), "$E:%s\r", msg);
    if (len <= 0)
    {
    	const char *error_msg = "ERROR: FSW attempted to send message with incorrect format";
    	len = snprintf(buffer, sizeof(buffer), "$E:%s\r", error_msg);
    }

    size_t to_send = (len < (int)sizeof(buffer)) ? (size_t)len : (sizeof(buffer) - 1);
    HAL_UART_Transmit(serialPort, (uint8_t*)buffer, (uint16_t)to_send, HAL_MAX_DELAY);
}

void SerialManager::sendInfoMsg(const char* msg)
{
    char buffer[RESP_SIZE];
    int len = snprintf(buffer, sizeof(buffer), "$I:%s\r", msg);
    if (len < 0)
    {
    	this->sendErrorMsg("ERROR: FSW attempted to send message with incorrect format");
    	return;
    }

    size_t to_send = (len < (int)sizeof(buffer)) ? (size_t)len : (sizeof(buffer) - 1);

    // blocking transmit:
    HAL_UART_Transmit(serialPort, (uint8_t*)buffer, (uint16_t)to_send, HAL_MAX_DELAY);
}


void SerialManager::sendErrorDataMsg(const char *format, ...)
{
	char buffer[RESP_SIZE];

    va_list args;
    va_start(args, format);
    int ret = vsnprintf(buffer, sizeof(buffer), format, args);

    if (ret < 0)
    {
        // formatting error
        this->sendErrorMsg("ERROR: FSW attempted to send message with incorrect format");
        return;
    }

    // If truncated, ret >= sizeof(buffer). Buffer is still NUL-terminated.
    if ((size_t)ret >= sizeof(buffer))
    {
        buffer[sizeof(buffer)-1] = '\0';
    }

    this->sendErrorMsg(buffer);
    va_end(args);
}

void SerialManager::sendInfoDataMsg(const char *format, ...)
{

    char buffer[RESP_SIZE];

    va_list args;
    va_start(args, format);

    int ret = vsnprintf(buffer, sizeof(buffer), format, args);

    if (ret < 0)
    {
		// formatting error
		this->sendErrorMsg("ERROR: FSW attempted to send message with incorrect format");
		return;
    }

	// If truncated, ret >= sizeof(buffer). Buffer is still NUL-terminated.
	if ((size_t)ret >= sizeof(buffer))
	{
		buffer[sizeof(buffer)-1] = '\0';
	}

    this->sendInfoMsg(buffer);

    va_end(args);
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

