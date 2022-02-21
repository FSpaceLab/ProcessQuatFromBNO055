/**
 * @file SpaceHandler.h
 * @author BohdanKorzhak (@Korzhak on GitHub)
 * @brief some functions for calculating angles from quaternion
 *        for controlling robot
 * @version 0.2.2 (Without functions for working with Euler's angles)
 * @date 2021-02-11
 * 
 * @copyright it's free for use
 * 
 */

#ifndef SPACEHANDLER_H
#define SPACEHANDLER_H

#define _USE_MATH_DEFINES
#include <math.h>
#include <Arduino.h>


// https://habr.com/ru/post/255005/


struct Quaternion {
    float w, x, y, z;
};    

struct Angles {
    float from_x, from_y, from_z, theta;
};

struct EulerAngles {
    float yaw, pitch, roll;
};


float quaternion_length(Quaternion q);

Quaternion create_quaternion_from_exist(float x, float y, float z, float angle);
Quaternion quaternion_scale(Quaternion q, float val);
Quaternion quaternion_normalize(Quaternion q);
Quaternion quaternion_invert(Quaternion q);
Quaternion quaternion_mult(Quaternion a, Quaternion b);        
Quaternion quaternion_div(Quaternion a, Quaternion b);

EulerAngles quaternion_to_euler(Quaternion q);
Angles get_angles_from_quat(Quaternion quat, int8_t x_dir, int8_t y_dir, int8_t z_dir);


#endif