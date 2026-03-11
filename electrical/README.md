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
- Digikey: [LTC3536](https://www.digikey.com/en/products/detail/analog-devices-inc/LTC3536EMSE-PBF/2720693?s=N4IgTCBcDaIDIBUDCBmArCgbAUQLIGVsBiABQCEAxAWgDkAREAXQF8g)
- Environment: CPL
- Details: 3.3v buckboost from 1s (3.7v) battery to 3.3v for the sensors on the CPL. 

### MP3424
- Datasheet: [MP3424A.pdf](MP3424A.pdf)
- Digikey: [MP3424A](https://www.digikey.com/en/products/detail/monolithic-power-systems-inc/MP3414AGJ-Z/7361472?s=N4IgTCBcDaIIwFYAcBOAtHAbAZgOwbQDkAREAXQF8g)
- Environment: CPL
- Details: 5.5v boost from 1s (3.7v) battery to 5.5v for the cameras, rtc clock and motors on the CPL.
- Node: To make use of the True Output Disconnect feature, the EN pin must be driven low. So **EN should be connected to Vin**. When battery connected, EN is high and the converter operates. When battery disconnected, EN is low and the converter is disabled and blocks current flow from the output to the input. 

### MAX756CPA+
- Datasheet: [max756.pdf](max756.pdf)
- Environment: CPL
- Details: Two exist
    - One is for 5v boost from 1s (1.2v) ni-mh battery to 5v for the buzzer circuit on the CPL.
    - Another will be used for 5v boost from 1s (3.7v) battery to 5v for the STM on the CPL. This is to allow for power isolation between the STM and the sensors, so we can power the STM without powering the sensors for debugging.


---

### General Notes
#### Power Isolation
To allow for debugging, the STM32 must be able to be powered without powering the sensors. To achieve this, we decide put the STM on a separate power rail from the sensors, so it will need a seperate boost (5v power option) or buckboost (3.3v power option) converter. We decide to go with 5v as we have an extra Max756CPA+ boost converter, which will be used to power the STM alone. To allow for debugging though, the sensors (and motors) must also be able to be powered without powering the STM. We need power isolation between all output rails. 

**We will assume that debugging always happens when the battery is disconnected (and has been for a couple seconds) and power is supplied to the stm through the usb, while power is supplied to the sensors and motors at the Vout nodes of the 3.3v and 5.5v converters.** 

No voltage should be present at the Vin terminals of the converters this **WILL** break stuff. So do not connect power to where the battery was connected, you must provide power to the sensors and motors using an external power supply connected to the test points on the PCB.

Below is the thought process for the power isolation design.

**MP3424A**:
- Synchronous boost with true output disconnect via internal PMOS. Actively blocks body diode conduction in shutdown according to datasheet.
- For this to work **EN must be tied to Vin**, that way when Vin is disconnected from the battery, EN is low and the converter is disabled and blocks current flow from the output to the input. When battery is connected, EN is high and the converter operates as normal.

**LTC3536**:
- 4-switch synchronous buck-boost with a direct reverse path while running
- If EN is high while Vout was driven, it has a reverse current limit (0.55A) to protect itself, but doesn't fully prevent backflow. We should NOT rely on this feature to prevent backflow, we should make sure converter is disabled when battery is disconnected.
- SHDN (shutdown) pin is active-high, so **tie SHDN to VIN**. Allows converter to operate when battery is connected, and disables it when battery is disconnected while preverting backflow.

**Max756CPA+**:
- Asynchronous boost with external discrete 1N5822 Schottky
- Schottky provides hard reverse blocking
- Reverse current at our expected voltage and temperatures is negligible.
![alt text](imgs/rev_i_graph_in5822.png) 

To ensure Vin is truely discharged while debugging, we will add a solder bridge across, from Vin to a pull down resistor to GND. This resistor must be a through hole (estimated value of 3.3kΩ) to ensure fast discharge of Vin when battery is disconnected. This will mean we will lose some current through the resistor, we may have to tune this value. **The solder bridge must only be connected while testing, it should be disconnected during normal operation.**

---

#### Board debugging procedure
There are two actors that require processing before board debugging. Follow in the order below:
1. Power isolation must be ensured as discussed in the previous section. To do this:
    - Disconnect battery and connect the solder bridge, ensuring Vin is pulled to ground through the pull down resistor.
    - Using a multimeter, verify that there is no voltage at the Vin terminals of the converters.
    - Provide power to the sensors via external power supply connected to the 3.3v and GND test point on the PCB.
    - Provide power to the motors and cameras via external power supply connected to the 5.5v and GND test point on the PCB.
    - Provide power to the STM via external power supply connected to the 5v and GND test point on the PCB. 
    - Now you can move to STM debugging setup sequence.
2. The STM must follow a specific power up sequence.
![alt text](imgs/stm_deb.png)