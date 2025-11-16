
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <Wire.h> 
#include <math.h>


SFE_UBLOX_GNSS myGNSS;


void setup() {
  
  Serial.begin(115200);

  Wire.begin();

  while(myGNSS.begin() == false){
    Serial.println("GNSS X, retrying..");
    delay(1000);
  } 
 
  Serial.println("GNSS OK");

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output both NMEA and UBX messages

}



void loop() {

myGNSS.checkUblox();

  Serial.println(">>> GPS Data Block >>>");
  Serial.println();

  long latitude = myGNSS.getLatitude();
    Serial.print(F("Latitude: "));
    Serial.println(latitude/1e7 , 4);

  long longitude = myGNSS.getLongitude();
    Serial.print(F("Longitude: "));
    Serial.println(longitude/1e7 , 4);
    
  long altitude = myGNSS.getAltitude();
    Serial.print(F("GPS Altitude: "));
    Serial.println(altitude/1e3 , 1);
     
q
  byte SIV = myGNSS.getSIV();
    Serial.print(F("# Satellites: "));
    Serial.println(SIV);

  

  Serial.print(F("UTC Time: " ));
  Serial.print(myGNSS.getHour()   < 10 ? "0" : ""); Serial.print(myGNSS.getHour());
  Serial.print(":");
  Serial.print(myGNSS.getMinute() < 10 ? "0" : ""); Serial.print(myGNSS.getMinute());
  Serial.print(":");
  Serial.print(myGNSS.getSecond() < 10 ? "0" : ""); Serial.println(myGNSS.getSecond());

  Serial.println();
  Serial.println("<<< GPS Data Block <<<");

  Serial.println();

  delay(1000);

}
