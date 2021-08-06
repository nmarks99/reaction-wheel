#ifndef PID_H_

#define INTEGRAL_TRIG 25

// Define PID constants
extern float kp;
extern float ki;
extern float kd;

float computePID(float theta, unsigned char a_motor);

#endif
