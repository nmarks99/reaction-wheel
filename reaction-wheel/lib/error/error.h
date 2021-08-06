#ifndef ERROR_H_
#include "RW.h"

float computeError(float theta, unsigned char a_motor);
float motorSign(float theta, float targetTheta);
int find_quad(float angle);

#endif