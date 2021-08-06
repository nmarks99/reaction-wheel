#include "error.h"
#include <stdlib.h>

int find_quad(float angle){
    int quad;
    if ((angle > 0) && (angle < 90)){
        quad = 1;
    }
    else if ((angle > 90) && (angle < 180)){
        quad = 2;
    }
    else if ((angle > 180) && (angle < 270)){
        quad = 3;
    }
    else{
        quad = 4;
    }
    return quad;
}


float motorSign(float theta, float targetTheta){
  
  float opp;  // Angle opposite the target angle
  unsigned char a_motor;

  if ((targetTheta >= 0) && (targetTheta <= 90)){ // Target in quadrant 1 (including 0 and 90 degrees)
    opp = targetTheta - 180;
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
  return a_motor;
}


/* Calculates the distance between the current angle and the target angle */
float computeError(float theta, unsigned char a_motor){
  
  float error;
  /* Error calculation when system needs to spin ccw (a_motor == 0) and target in quad 1 */
  if (a_motor == 0){
    if (find_quad(theta) == 1){
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
    if (find_quad(theta) == 1){
      if (theta > 0){
        error = targetTheta - theta;
      }
      else if (theta < 0){
        error = abs(theta - targetTheta);
      }
    }

    
  } //END case where a_motor == 1
  
  return error;
} //END FUNCTION