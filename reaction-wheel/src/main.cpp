#include "RW.h"
#include "PID.h"
#include "error.h"

const int speedPin = 6;
const int cw = 7;
const int ccw = 8;

float targetTheta; // Target position (degrees)
float devTheta = 2.0; // Acceptable deviation from target
unsigned char idlePWM = 50; //50
unsigned char maxPWM = 85; //85

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
SoftwareSerial BT(4, 5); // RX,TX

void setup() {

  /* Begin serial communication for bluetooth module */
  BT.begin(9600);

  /* Give some values to variables */
  idlePWM = 50; //50
  targetTheta = 1.0;
  devTheta = 2.0;
  maxPWM = 85; //85

  /* Initialize motor control pins */
  pinMode(speedPin, OUTPUT);
  pinMode(cw, OUTPUT);
  pinMode(ccw, OUTPUT);

  /* Set motor to idle */
  // PWMval = idlePWM;
  analogWrite(speedPin, idlePWM);
  digitalWrite(ccw, HIGH);

  /* Initialize the IMU */
  if (!bno.begin())
  {
    /* BNO055 not detected */
    BT.print("No BNO055 detected. Check wiring");
    while (1)
      ; // Loops forever doing nothing if BNO055 is not detected
  }

  /* Wait for motor to reach idle speed */
  delay(5000);
}


void loop() {
  
  /* Get Euler angle vectors from the IMU */
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

   /* Get current angular position (x axis Euler angle) and restrict domain between -180 and 180, (CW+, CCW-) */
  float theta = euler.x();  
  if (theta > 180){
    theta = theta - 360;
  }

   /* Print some values over Bluetooth */
  BT.print(theta);
  BT.print(",");
  BT.print(millis());
  BT.println("");

  /* Run functions to compute current error and required direction of the motor's acceleration */
  float a_motor = motorSign(targetTheta,theta); 
  float error = computeError(theta,a_motor);

  delay(LOOP_DELAY_MS);

  /* While the current angle is outside the target range, run feedback loop to bring system back to target */
  while (error > devTheta){

    /* Get current position, same at before */
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    theta = euler.x();
    if (theta > 180){
      theta = theta - 360;
    }
    
    /* Calculate the direction motor needs to spin, the error, then calculate PWMval (PID output) with PID controller */
    a_motor = motorSign(theta,targetTheta);
    error = computeError(theta,a_motor);
    float PWMval = computePID(theta,a_motor);
    
    /* Reaction wheel will spin up or spin down accordingly */
    if (a_motor == 0){
      PWMval = idlePWM - PWMval;
    }
    else if (a_motor == 1){
      PWMval = idlePWM + PWMval;
    }

    /* Restrict the PWM value to be postive and less than the pre-defined maximum */
    if (PWMval > maxPWM){
      PWMval = maxPWM;
    }
    else if (PWMval < 0){
      PWMval = 0;
    }

    /* Send new PWM value to motor */
    analogWrite(speedPin,PWMval);

    /* Print some values over Bluetooth */
    BT.print(theta);
    BT.print(",");
    BT.print(millis());
    BT.println("");   
    
    delay(LOOP_DELAY_MS);
}

}