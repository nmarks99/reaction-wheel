#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SoftwareSerial.h>

/* Define pins */
int speedPin = 6;
int cw = 7;
int ccw = 8;
// Connect SCL from BNO055 to A5
// Connect SDA from BNO055 to A4

/* Initialize some variables */
float PWMval; // PWM value sent to motor, output from PID controller
float idlePWM;  // PWM value for idle speed
float maxPWM; // Maximum allowable PWM value
float theta;  // Angular position of system (x-axis Euler angle)
float delay_time; // Delay time in loop
int target_quad;  // Quadrant target position is located in
float targetTheta;  // Target position
float devTheta; // Allowable deviation from the target 
int a_motor;  // Flag to assign motor acceleration direction. Either 0 (sys needs to go ccw) or 1 (sys needs to go cw) 
float error;  // Distance between current postion and target position

/* Define the BNO055 IMU */
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

/* Setup bluetooth connection */
SoftwareSerial BT(4, 5); // RX,TX


/**************************************************************************/
/*
    Function to calculate error (distance betweeen current and target position
    TODO: Add case where target position is not in quadrant 1
*/
/**************************************************************************/


/* Calculates the distance between the current angle and the target angle */
float computeError(){
  
  /* Error calculation when system needs to spin ccw (a_motor == 0) and target in quad 1 */
  if (a_motor == 0){
    if (target_quad == 1){
      if (theta > 0){
        error = theta - targetTheta;
      }
      else if (theta < 0){
        error = ((180 - targetTheta) + (180 + theta));
      }
    }
    
  } // END case where a_motor == 0

  /* Error calculation when system needs to spin cw (a_motor == 1) and target in quad 1 */
  else if (a_motor == 1){
    if (target_quad == 1){
      if (theta > 0){
        error = targetTheta - theta;
      }
      else if (theta < 0){
        error = abs(theta - targetTheta);
      }
    }

    
  }//END case where a_motor == 1
  
  return error;
}//END FUNCTION


/**************************************************************************/
/*
    Setup PID controller
*/
/**************************************************************************/

/* Define PID constants */
float kp = 0.5;
float ki = 0.1;
float kd = 3.5;

float intError, drvError, lastError;
int intCount = 0;

float computePID(){     
  
  /* Calculate errors */
  error = computeError();  // Error = distance from target position

  /* Only accumulate intergral error when the proportional error is small */
  if (error < 25){
    intError += error;  // Integral error
    intCount ++;
    if (intCount > 10){
      intError = 0;
      intCount = 0;         
    }
  }
  else{
    intError = 0;               
  }
  
  drvError = (error - lastError); // Derivative error  

  /* Calculate output (PWM value) */
  PWMval = kp*error + ki*intError + kd*drvError;

  /* Store previous error */
  lastError = error;   

  /* Return the PWM valute */
  return PWMval;
                                 
}


/**************************************************************************/
/*
    Arduino setup function 
*/
/**************************************************************************/
void setup(void){

  /* Begin serial communication for bluetooth module */
  BT.begin(9600);
  
  /* Give some values to variables */
  idlePWM = 50; //50
  targetTheta = 1.0;
  devTheta = 2.0; 
  delay_time = 50;
  maxPWM = 85; //85
 
  /* Initialize motor control pins */
  pinMode(speedPin,OUTPUT);
  pinMode(cw,OUTPUT);
  pinMode(ccw,OUTPUT);

  /* Set motor to idle */
  PWMval = idlePWM;
  analogWrite(speedPin,idlePWM);
  digitalWrite(ccw,HIGH);
  
  /* Initialize the IMU */
  if(!bno.begin())
  {
    /* BNO055 not detected */
    BT.print("No BNO055 detected. Check wiring");
    while(1); // Loops forever doing nothing if BNO055 is not detected
  }

  /* Wait for motor to reach idle speed */
  delay(5000);
  
}//END SETUP



/**************************************************************************/
/*
    Function to determine the required direction of the motor's accerlation
*/
/**************************************************************************/

void motorSign(){
  
  int opp;  // Angle opposite the target angle

  if ((targetTheta >= 0) && (targetTheta <= 90)){ // Target in quadrant 1 (including 0 and 90 degrees)
    opp = targetTheta - 180;
    target_quad = 1;
    if ((theta < 0) && (theta < opp)){
      a_motor = 0;   // want sys to spin ccw, 0 means negative motor accerlation
    }
    else if ((theta > targetTheta) && (theta < 180)){
      a_motor = 0;  // want sys to spin ccw, 0 means negative motor acceleration
    }
    else{
      a_motor = 1;  // want sys to spin cw, 1 means positive motor acceleration
    }
  }

  else if ((targetTheta < 0) && (targetTheta > -90)){ // Target in quadrant 2
    opp = targetTheta + 180;
    target_quad = 2;
    if ((theta > 0) && (theta < opp)){
      a_motor = 0;  // want sys to spin ccw, 1 means negative motor acceleration 
    }
    else if ((theta < 0) && (theta > targetTheta)){
      a_motor = 0;  // want sys to spin ccw
    }
    else{
      a_motor = 1;
    }
  }

  else if ((targetTheta < -90) && (targetTheta > -180)){ // Target in quadrant 3
    opp = targetTheta + 180;
    target_quad = 3;
    if ((theta > 0) && (theta < opp)){
      a_motor = 0;  // want sys to spin ccw
    }
    else if ((theta < 0) && (theta > targetTheta)){
      a_motor = 0;  // want sys to spin ccw
    }
    else{
      a_motor = 1;  // want sys to spin cw
    }
  }

  else{  // Target is in quadrant 4
    opp = targetTheta - 180;
    target_quad = 4;
    if ((theta > 0) && (theta > targetTheta)){
      a_motor = 0;  // want sys to spin ccw
    }
    else if ((theta < 0) && (theta < opp)){
      a_motor = 0;  // want sys to spin ccw
    }
    else{
      a_motor = 1;  // want sys to spin cw
    }
  }
}



/**************************************************************************/
/*
    Arduino loop function
*/
/**************************************************************************/
void loop(void){
  
  /* Get Euler angle vectors from the IMU */
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  
  /* Get current angular position (x axis Euler angle) and restrict domain between -180 and 180, (CW+, CCW-) */
  theta = euler.x();  
  if (theta > 180){
    theta = theta - 360;
  }
  
  /* Print some values */
  BT.print(theta);
  BT.print(",");
  BT.print(millis());
  BT.println("");

  /* Run functions to compute current error and required direction of the motor's acceleration */
  motorSign(); 
  error = computeError();
  
  delay(delay_time);
  
  /* While the current angle is outside the target range, run feedback loop to bring system back to target */
  while (error > devTheta){

    /* Get current position, same at before */
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    theta = euler.x();
    if (theta > 180){
      theta = theta - 360;
    }
    
    /* Calculate the direction motor needs to spin, the error, then calculate PWMval (PID output) with PID controller */
    motorSign();
    error = computeError();
    PWMval = computePID();
    
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

    /* Print some values */
    BT.print(theta);
    BT.print(",");
    BT.print(millis());
    BT.println("");   
    
    delay(delay_time);
}
  
}//END LOOP
