#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>                 // AHT21 library
#include "ScioSense_ENS160.h"               // ENS160 library
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

// ROS setup
ros::NodeHandle nh;
std_msgs::Float32MultiArray msg_arr;

// save all the values to array: PM2.5, PM10, CO2, VO2, temp, humidity, CO, NH3, NO2
float value[9] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

ros::Publisher Pub_arr("/data", &msg_arr);

// Define the length of the information being retrieved from the PMS5003 sensor
#define LENG 24
#define TIME_BETWEEN_READINGS 1000          // delay for 1s

// MiCS-6814 (CO, NH3, NO2)
const int CO_PIN = A0;
const int NH3_PIN = A2;
const int NO2_PIN = A4;

// PMS5003 (PM2.5, PM10)
unsigned char buf[LENG];                    // array for PMS5003 sensor
unsigned short PM25 = -1;                   // variable for PM2.5
unsigned short PM10 = -1;                   // variable for PM10

int transmitPM2_5(unsigned char *thebuf) {
    return ((thebuf[5] << 8) + thebuf[6]);  // calculate PM2.5
}
int transmitPM10(unsigned char *thebuf) {
    return ((thebuf[7] << 8) + thebuf[8]);  // calculate PM10
}

SoftwareSerial PMSerial(14, 13);            // PMS5003 serial (RX=14, TX=13)

// AHT21 (temp, humidity)
Adafruit_AHTX0 aht;
int tempC;                                  // temp (Celsius)
int humidity;

// ENS160 (CO2, VOC)
ScioSense_ENS160 ens160(ENS160_I2CADDR_1);

void setup() {
    // ROS initizlization
    nh.initNode();
    nh.advertise(Pub_arr);

    // wait for ROS node to connect
    while (!nh.connected()) {
        nh.spinOnce();
    }

    // initizlize all the sensors
    // PMS5003
    PMSerial.begin(9600);
    Serial.begin(57600);
    while (!Serial) {;}
    Serial.println("Starting PMS5003...");

    // AHT21
    Serial.print("AHT21...");
    if (aht.begin()) {
        Serial.println("found.");
    } else {
        Serial.println("failed! Check wiring.");
        while (1) delay(10);
    }

    // ENS160
    Serial.print("ENS160...");
    delay(500);
    if (ens160.begin()) {
        Serial.println("done.");
        Serial.print("\tRev: "); Serial.print(ens160.getMajorRev());
        Serial.print("."); Serial.print(ens160.getMinorRev());
        Serial.print("."); Serial.println(ens160.getBuild());
        Serial.print("\tStandard mode ");
        Serial.println(ens160.setMode(ENS160_OPMODE_STD) ? "done." : "failed!");
    } else {
        Serial.println("failed! Please check connections and I2C address.");
    }
}

void loop() {
    // PMS5003
    if (PMSerial.find(0x42)) {
        PMSerial.readBytes(buf, LENG);
        if (buf[0] == 0x4d) {
            PM25 = transmitPM2_5(buf);
            PM10 = transmitPM10(buf);
        }
    }

    // AHT21
    sensors_event_t humidity1, temp;
    aht.getEvent(&humidity1, &temp);
    tempC = temp.temperature;
    humidity = humidity1.relative_humidity;

    // ENS160
    if (ens160.available()) {
        ens160.set_envdata(tempC, humidity);
        ens160.measure(true);
        ens160.measureRaw(true);
    }

    // MiCS-6814
    int coValue = analogRead(CO_PIN);
    int nh3Value = analogRead(NH3_PIN);
    int no2Value = analogRead(NO2_PIN);

    float coVoltage = coValue * (3.3 / 4096.0);
    float nh3Voltage = nh3Value * (3.3 / 4096.0);
    float no2Voltage = no2Value * (3.3 / 4096.0);

    // save all the values to array: PM2.5, PM10, CO2, VO2, temp, humidity, CO, NH3, NO2
    value[0] = PM25;
    value[1] = PM10;
    value[2] = ens160.geteCO2();  
    value[3] = ens160.getTVOC();  
    value[4] = tempC;              
    value[5] = humidity;           
    value[6] = coVoltage;            
    value[7] = nh3Voltage;           
    value[8] = no2Voltage;
    
    // print the result (Debugging)
    // Serial.println("Sensor Values:");
    // for (int i = 0; i < 8; i++) {
    //     Serial.print(value[i]); Serial.print(", ");
    // }
    // for (int i = 8; i < 9; i++) {
    //     Serial.println(value[i]);
    // }

    // read data from the sensors and upload to
    msg_arr.data = value;
    msg_arr.data_length = 9;
    Pub_arr.publish(&msg_arr);

    nh.spinOnce();

    delay(TIME_BETWEEN_READINGS);            // delay for 1 sec
}
