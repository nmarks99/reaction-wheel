#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SoftwareSerial.h>
#include <stdio.h>
#include <stdlib.h>

#define LOOP_DELAY_MS 50

// Define pins 
extern const int speedPin; 
extern const int cw;
extern const int ccw;
// Connect SCL from BNO055 to A5
// Connect SDA from BNO055 to A4

extern float targetTheta; // Target position (degrees)
extern float devTheta; // Acceptable deviation from target
extern unsigned char idlePWM; //50
extern unsigned char maxPWM; //85

/* Define the BNO055 IMU object*/ 
extern Adafruit_BNO055 bno;

/* Setup bluetooth connection */
extern SoftwareSerial BT; // RX,TX