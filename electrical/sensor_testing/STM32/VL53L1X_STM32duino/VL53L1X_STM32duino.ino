#include <Wire.h>
#include <vl53l1x_class.h>

#define XSHUT_PIN  PA4        // VL53L1X XSHUT (gpio0 for the class) (not actually doing anything)

// Constructor: (TwoWire *i2c, int gpio0) -> use XSHUT here
VL53L1X vl53(&Wire, XSHUT_PIN);

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    // wait for usb serial
  }

  Serial.println("STM32duino VL53L1X test");
  
  Wire.begin();
  vl53.VL53L1X_On();

  VL53L1X_ERROR status;
  uint16_t configInfo;

  status = vl53.VL53L1X_SensorInit();
  Serial.print("SensorInit status = ");
  if (status != 0) {
    Serial.println("SensorInit failed");
    while (1) delay(1000);
  }

  status = vl53.VL53L1X_SetDistanceMode(2);
  if (status != 0) {
    Serial.println("SetDistanceMode error");
  } else {
    vl53.VL53L1X_GetDistanceMode(&configInfo);
    Serial.print("DistanceMode = ");
    Serial.println(configInfo);
  }

  // Timing budget & inter-measurement time (in ms). Valid budgets: 15–500 ms.[web:51]
  status = vl53.VL53L1X_SetTimingBudgetInMs(50);
  if (status != 0) {
    Serial.println("SetTimingBudget error");
  } else {
    vl53.VL53L1X_GetTimingBudgetInMs(&configInfo);
    Serial.print("TimingBudget = ");
    Serial.print(configInfo);
    Serial.println(" ms");
  }

  status = vl53.VL53L1X_SetInterMeasurementInMs(60);
  if (status != 0) {
    Serial.println("SetInterMeasurement error");
  } else {
    vl53.VL53L1X_GetInterMeasurementInMs(&configInfo);
    Serial.print("InterMeasurement = ");
    Serial.print(configInfo);
    Serial.println(" ms");
  }

  status = vl53.VL53L1X_SetROI(16, 16);
  if (status != 0) {
    Serial.println("SetROI error");
  } else {
    Serial.println("ROI set");
  }

  Serial.println("-----------------------");

  int16_t offset;

  vl53.VL53L1X_GetOffset(&offset);
  Serial.print("Offset = ");
  Serial.println(offset);

  vl53.VL53L1X_GetDistanceThresholdWindow(&configInfo);
  Serial.print("ThresholdWindow = ");
  Serial.println(configInfo);

  uint16_t ROI_X;
  uint16_t ROI_Y;

  vl53.VL53L1X_GetROI_XY(&ROI_X, &ROI_Y);
  Serial.print("ROI_X = ");
  Serial.println(ROI_X);
  Serial.print("ROI_Y = ");
  Serial.println(ROI_Y);

  Serial.println("-----------------------");

  status = vl53.VL53L1X_StartRanging();
  if (status != 0) {
    Serial.println("StartRanging error");
  } else {
    Serial.println("StartRanging started");
  }
}

void loop() {
  VL53L1X_ERROR status;
  uint8_t dataReady = 0;

  // block until a measurement is ready
  do {
    status = vl53.VL53L1X_CheckForDataReady(&dataReady);
    if (status != 0) {
      Serial.print("CheckForDataReady error = ");
      Serial.println(status);
      return;
    }
  } while (dataReady == 0);

  uint16_t distance = 0;
  uint8_t rangeStatus = 0;

  status = vl53.VL53L1X_GetRangeStatus(&rangeStatus);
  if (status != 0) {
    Serial.print("GetRangeStatus error = ");
    Serial.println(status);
  }

  status = vl53.VL53L1X_GetDistance(&distance);
  if (status != 0) {
    Serial.print("GetDistance error = ");
    Serial.println(status);
  }

  // Clear interrupt for next measurement
  vl53.VL53L1X_ClearInterrupt();

  if (rangeStatus == 0) {
    Serial.print("RangeStatus = ");
    Serial.print(rangeStatus);
    Serial.print("  Distance = ");
    Serial.print(distance);
    Serial.println(" mm");
  } else {
    Serial.println("Out of range");
  }
  
}
