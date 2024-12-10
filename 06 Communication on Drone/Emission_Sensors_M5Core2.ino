#include <M5Core2.h>
#include <Wire.h>
#include <SD.h>
#include "DFRobot_AirQualitySensor.h"     // SEN0460 library
#include "DFRobot_MultiGasSensor.h"       // SEN0466, 0470, 0471 library

File myFile;

// SEN0460 (PM1.0, PM2.5, PM10)
#define I2C_ADDRESS_PARTICLE    0x19
DFRobot_AirQualitySensor particle(&Wire, I2C_ADDRESS_PARTICLE);

// SEN0159 (CO2)
#define MG_PIN 36                         // analog input pin
#define BOOL_PIN 2                        // digital pin
#define DC_GAIN 8.5                       // DC gain

#define ZERO_POINT_VOLTAGE (0.258)        // voltage at 400 ppm (voltage/8.5)
#define REACTION_VOLTAGE (0.059)          // voltage at 1000 ppm (voltage/8.5)

float CO2Curve[3] = {2.602, ZERO_POINT_VOLTAGE, (REACTION_VOLTAGE / (2.602 - 3))};

float MGRead(int mg_pin) {
  int i;
  float v = 0;

  for (i = 0; i < 5; i++) {               // sampling for 5 times
    v += analogRead(mg_pin);
    delay(50);                            // delay for 50ms
  }
  v = (v / 5) * 3.25 / 4096;              // calculate the voltage
  return v;
}

int MGGetPercentage(float volts, float *pcurve) {
  volts = volts / DC_GAIN;
  if (volts >= ZERO_POINT_VOLTAGE) {
    return -1;                            // when CO2 is lower than 400 ppm, value is -1
  } else {
    return pow(10, (volts - pcurve[1]) / pcurve[2] + pcurve[0]);
  }
}

// SEN0466 (CO), SEN0470 (SO2), SEN0471 (NO2): SEL(0)
#define I2C_ADDRESS_CO    0x77            // A0 A1: 11
#define I2C_ADDRESS_SO2   0x74            // A0 A1: 00
#define I2C_ADDRESS_NO2   0x75            // A0 A1: 01
DFRobot_GAS_I2C gas_CO(&Wire, I2C_ADDRESS_CO);
DFRobot_GAS_I2C gas_SO2(&Wire, I2C_ADDRESS_SO2);
DFRobot_GAS_I2C gas_NO2(&Wire, I2C_ADDRESS_NO2);

// save all the values to array: PM1.0, PM2.5, PM10, CO2, CO, SO2, NO2
float value[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

void setup() {
  // Reset M5Core2
  M5.begin();
  Serial.begin(115200);
  Wire.begin(32, 33);   // PA_SDA: 32, PA_SCL: 33

  // Reset the SD card
  if (!SD.begin()) {
    Serial.println("Failed to reset SD card!");
    return;
  }
  Serial.println("Reset SD card");

  // initialize all the sensors
  // SEN0460 (PM)
  while (!particle.begin()) {
    Serial.println("NO Devices for SEN0460 !");
    delay(1000);
  }
  Serial.println("SEN0460 connected");

  // SEN0159 (CO2)
  pinMode(BOOL_PIN, INPUT);               // set the digital pin of sensor
  digitalWrite(BOOL_PIN, HIGH);           // activate the pull up resistor

  // SEN0466 (CO)
  while (!gas_CO.begin()) {
    Serial.println("NO Devices for SEN0466 !");
    delay(1000);
  }
  Serial.println("SEN0466 connected");

  // SEN0470 (SO2)
  while(!gas_SO2.begin()) {
    Serial.println("No Devices for SEN0470 !");
    delay(1000);
  }
  Serial.println("SEN0470 connected");

  // SEN0471 (NO2)
  while(!gas_NO2.begin()) {
    Serial.println("No Devices for SEN0471 !");
    delay(1000);
  }
  Serial.println("SEN0471 connected");

  delay(1000);

  // for SEN0466, 0470, 0471 sensor
  gas_CO.setTempCompensation(gas_CO.ON);
  gas_SO2.setTempCompensation(gas_SO2.ON);
  gas_NO2.setTempCompensation(gas_NO2.ON);
}

void loop() {
  // SEN0460 (PM)
  uint16_t num1 = particle.gainParticleNum_Every0_1L(PARTICLENUM_1_0_UM_EVERY0_1L_AIR);
  uint16_t num25 = particle.gainParticleNum_Every0_1L(PARTICLENUM_2_5_UM_EVERY0_1L_AIR);
  uint16_t num10 = particle.gainParticleNum_Every0_1L(PARTICLENUM_10_UM_EVERY0_1L_AIR);
  Serial.print("PM1.0: "); Serial.print(num1); Serial.print(", ");
  Serial.print("PM2.5: "); Serial.print(num25); Serial.print(", ");
  Serial.print("PM10: "); Serial.println(num10);

  // SEN0159 (CO2)
  int percentage;
  float volts = MGRead(MG_PIN);

  percentage = MGGetPercentage(volts, CO2Curve);
  Serial.print("CO2: ");
  if (percentage == -1) {
    Serial.print("<400");
  } else {
    Serial.print(percentage);
  }
  Serial.println(" ppm");

  // SEN0466 (CO)
  Serial.print(gas_CO.queryGasType());
  Serial.print(": ");
  Serial.println(gas_CO.readGasConcentrationPPM());

  // SEN0470 (SO2)
  Serial.print(gas_SO2.queryGasType());
  Serial.print(": ");
  Serial.println(gas_SO2.readGasConcentrationPPM());

  // SEN0471 (NO2)
  Serial.print(gas_NO2.queryGasType());
  Serial.print(": ");
  Serial.println(gas_NO2.readGasConcentrationPPM());
  Serial.println("----------");

  // save all the values to array: PM1.0, PM2.5, PM10, CO2, CO, SO2, NO2
  value[0] = num1;
  value[1] = num25;
  value[2] = num10;
  value[3] = percentage;
  value[4] = gas_CO.readGasConcentrationPPM();
  value[5] = gas_SO2.readGasConcentrationPPM();
  value[6] = gas_NO2.readGasConcentrationPPM();

  // Save data every 5 second
  static unsigned long lastSaveTime = 0;
  if (millis() - lastSaveTime >= 5000) {
    lastSaveTime = millis();    // latest save time

    // Open the file from the SD card (If there's no file, then make a new one)
    myFile = SD.open("/sensor_data.txt", FILE_WRITE);
    if (myFile) {
      // Write the sensor data to the SD card
      for (int i = 0; i < 7; i++) {
        myFile.print(value[i]);
        if (i < 6) {
          myFile.print(", ");
        } else {
          myFile.println();
        }
      }
      myFile.close();  // Close the file after writing
      Serial.println("Data saved to SD card");
      Serial.println("--------------------");
    } else {
      Serial.println("Failed to open the file!");
    }
  }
    
  // print the result (Debugging)
  Serial.println("Sensor Values:");
  for (int i = 0; i < 7; i++) {
    Serial.print(value[i]);
    if (i < 6) {
        Serial.print(", ");
    } else {
        Serial.println();
    }
  }
  Serial.println("--------------------");
  
  delay(1000);                            // delay for 1 sec
}
