/*
 * Helper Functions
*/
#ifndef _helper_H_
#define _helpter_H_
#include <Arduino.h>
// #include "config.h"
void loopRate(int freq,float functionstarttime);
float invSqrt(float x);
float angular_diff(float target_angle, float source_angle);
float saturate(float input, float upperbound, float lowerbound);
float correct_heading_wrap(float current_heading);
#endif
