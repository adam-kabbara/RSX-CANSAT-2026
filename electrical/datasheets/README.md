@adamlkabbara to fix this 

# Datasheets of electronic components
This folder contains datasheets of electronic components used in the project. This readme provides notes and highlights of the datasheets for quick reference.

## I2C Adress Book
- INA219 (A0, A1 are gnd on CPL): 0x40
- BMP581: 0x47 (can be changed to 0x46)
- BNO085: 0x4A (can be changed to 0x4B)
- MAXM10S: 0x42 
- VL53L1-SATEL​: 0x29

## I/O
### EEPROM
- Datasheet: [EEPROM.pdf](EEPROM.pdf)
- Environment: CPL

### INA219
- Datasheet: [ina219.pdf](ina219.pdf)
- Environment: Ground station and CPL
- Package: D ; using smd on cpl and adafruit on gnd station

## Power Electronics
### LT8610AB
- Datasheet: [LT8610AB.pdf](LT8610AB.pdf)
- Environment: Ground station
- Details: 5v buck converter from 4s2p (16.8 to 12v) battery to 5v for the ground station. 5 volts used to power the raspberry pi and LED lights. Estimated current consumption to handel is 2.5A.

### LTC3114
- Datasheet: [LTC3114.pdf](LTC3114.pdf)
- Environment: Ground station
- Details: 12v buckboost from 4s2p (16.8 to 12v) battery to 12v for the ground station. 12 volts used to power the monitor and speakers.
- Package: FE

### LTC3536
- Datasheet: [LTC3536.pdf](LTC3536.pdf)
- Environment: CPL
- Details: 3.3v buckboost from 1s (3.7v) battery to 3.3v for the sensors on the CPL. 
