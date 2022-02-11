#include "SpaceHandler.h"

/**
 * @brief Function for creating structure which contains x-z and w values of quaternion
 * 
 * @param x X value of exist Quaternion
 * @param y Y value of exist Quaternion
 * @param z Z value of exist Quaternion
 * @param angle W value of exist Quaternion
 * @return structure of Quaternion with passed values
 */
Quaternion create_quaternion_from_exist(float x, float y, float z, float angle) {
    Quaternion quat;
    
    quat.w = angle;
    quat.x = x;
    quat.y = y;
    quat.z = z;
    return quat;
}


Quaternion quaternion_scale(Quaternion q, float val) {
    q.w *= val;
    q.x *= val;
    q.y *= val;
    q.z *= val;
    return q;
}


double quaternion_length(Quaternion q) {
    return pow(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z, 0.5);
}


Quaternion quaternion_normalize(Quaternion q) {
    double n = quaternion_length(q);
    return quaternion_scale(q, 1/n);
}


Quaternion quaternion_invert(Quaternion q) {
    Quaternion res;
    res.w = q.w;
    res.x = -q.x;
    res.y = -q.y;
    res.z = -q.z;
    return quaternion_normalize(res);
}


Quaternion quaternion_mult(Quaternion a, Quaternion b) {
    Quaternion res;
    res.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    res.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    res.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    res.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
    return res;
}

/** 
 * @brief Calculating angles for robot from quaternion
 * 
 * @param quat Exist quaternion
 * @param x_dir 1 or -1 (int8_t) value for changing the direction of X axis
 * @param y_dir 1 or -1 (int8_t) value for changing the direction of Y axis
 * @param z_dir 1 or -1 (int8_t) value for changing the direction of Z axis
 * @return structure of Angles with values of x, y and z axises
 */
Angles get_angles_from_quat(Quaternion quat, int8_t x_dir, int8_t y_dir, int8_t z_dir) {
    Angles angles;

    angles.from_x = x_dir * 2 * atan2(quat.x, quat.w) * (180.0/3.141592653589793238463);
    angles.from_y = y_dir * 2 * atan2(quat.y, quat.w) * (180.0/3.141592653589793238463);
    angles.from_z = z_dir * 2 * atan2(quat.z, quat.w) * (180.0/3.141592653589793238463);
    // angles.theta  = 2 * atan2(sqrt(quat.x * quat.x + quat.y * quat.y + quat.z * quat.z), quat.w) * (180.0/3.141592653589793238463);  

    return angles;
}