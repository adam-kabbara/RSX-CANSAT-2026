/*
 * serialManager.h
 *
 *  Created on: Oct 11, 2025
 *      Author: avaniyadav
 */

#ifndef INC_SERIAL_MANAGER_H_
#define INC_SERIAL_MANAGER_H_


/* add classes and functions here
 */

class SerialManager
{
private:
    HardwareSerial* serialPort;

public:
    SerialManager(HardwareSerial& port)
        : serialPort(&port)
        {}

    void begin();

    int get_data(char *cmd_buff);

    void sendErrorMsg(const char *msg);

    void sendInfoMsg(const char *msg);

    void sendErrorDataMsg(const char *format, ...);

    void sendInfoDataMsg(const char *format, ...);

    void sendTelemetry(char *buff);

    void sendLogFile(File log);

};


#endif /* INC_SERIAL_MANAGER_H_ */
