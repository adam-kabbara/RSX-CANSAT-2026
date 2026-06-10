#include <SoftwareSerial.h>

// #ngl you can lowkey copy most of these functions into the STM32 code and you only have to replace the Serial object/functions
//Protocol Info:  https://support.runcam.com/hc/en-us/articles/360014537794-RunCam-Device-Protocol
//Manual:         https://www.runcam.com/download/split4k/RC_Split_4k_Manual_EN.pdf

#define RX 3    //RX connected to RunCam UART's TX pin
#define TX 2    //TX connected to RunCam UART's RX pin
#define CAMERA_COMMAND_ID 0x01 //ID for all camera-related commands (refer to protocol info)

SoftwareSerial runcamSerial(RX, TX); //uart object for RunCam Split 4-25

//Send command that takes in 4 bytes (including crc)
void send4ByteCommand(uint8_t ID, uint8_t cmd)
{
  byte data[4] = {0xCC, ID, cmd, 0x00};
  data[3] = compound_crc(data, 3);
  runcamSerial.write(data, 4);
}

//Send camera command (ID = 0x01)
void sendCameraCommand(uint8_t cmd)
{
  send4ByteCommand(CAMERA_COMMAND_ID, cmd);
}

void setup() {

  Serial.begin(9600);

  //3 byte command to get device info (testing for correct connectinos)
  byte data[3] = {0xCC, 0x00, 0x00};
  data[2] = compound_crc(data, 2);

  //init serial
  runcamSerial.begin(115200);

  Serial.println(String("Requesting data with crc = ") + data[2]);
  runcamSerial.write(data, 3);

  delay(100); //add small delay between serial writes and reads to prevent timing-related bugs 

  while(runcamSerial.available() > 0)
  {
    Serial.print(runcamSerial.read(), HEX);
    Serial.print(" ");
  }
  Serial.print('\n');

  delay(500);

  sendCameraCommand(0x01); //toggle (start) recording 
  delay(3000);
  sendCameraCommand(0x01); //toggle (stop) recording
}


void loop() {

}


//crc function provided by protocol info
uint8_t crc8_dvb_s2(uint8_t crc, unsigned char a)
{
    crc ^= a;
    for (int ii = 0; ii < 8; ++ii) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0xD5;
        } else {
            crc = crc << 1;
        }
    }

    return crc;
}

uint8_t compound_crc(byte* data, int len)
{
  uint8_t crc = 0;
  for(int i = 0; i < len; i++)
  {
    crc = crc8_dvb_s2(crc, data[i]);
  }

  return crc;
}
