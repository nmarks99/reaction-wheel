#include "PID.h"
#include "error.h"

float kp = 0.5;
float ki = 0.1;
float kd = 3.5;

float computePID(float theta, unsigned char a_motor){

    float intError = 0; 
    float drvError = 0; 
    float lastError = 0;
    int intCount = 0;
    float PWMval;

    /* Calculate errors */
    float error = computeError(theta,a_motor); // Error = distance from target position

    /* Only accumulate intergral error when the proportional error is small */
    if (error < INTEGRAL_TRIG){
        intError += error; // Integral error
        intCount++;
        if (intCount > 10)
        {
            intError = 0;
            intCount = 0;
        }
    }
    else{
        intError = 0;
    }

    drvError = (error - lastError); // Derivative error

    /* Calculate output (PWM value) */
    PWMval = kp * error + ki * intError + kd * drvError;

    /* Store previous error */
    lastError = error;

    /* Return the PWM value */
    return PWMval;
}