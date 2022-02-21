#include "SpaceHandler.h"

using namespace std;

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


/**
 * @brief Scalling quaternion 
 * 
 * @param q exist quaternion
 * @param val scaling value
 * @return scalled Quaternion 
 */
Quaternion quaternion_scale(Quaternion q, float val) {
    q.w *= val;
    q.x *= val;
    q.y *= val;
    q.z *= val;
    return q;
}


/**
 * @brief Calculating length of quaternion
 * 
 * @param q exist quaternion
 * @return length of quaternion (double) 
 */
float quaternion_length(Quaternion q) {
    return pow(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z, 0.5);
}


/**
 * @brief Normalizing quaternion (length of quaternion will be equal 1)
 * 
 * @param q exist quaternion
 * @return normalized Quaternion 
 */
Quaternion quaternion_normalize(Quaternion q) {
    double n = quaternion_length(q);
    return quaternion_scale(q, 1/n);
}

/**
 * @brief Inverting of quaternion
 * 
 * @param q exist quaternion
 * @return inverted Quaternion 
 */
Quaternion quaternion_invert(Quaternion q) {
    Quaternion res;
    res.w = q.w;
    res.x = -q.x;
    res.y = -q.y;
    res.z = -q.z;
    return quaternion_normalize(res);
}

/**
 * @brief Multiplication of two quaternions
 * 
 * @param a first quaternion
 * @param b second quaternion
 * @return result Quaternion 
 */
Quaternion quaternion_mult(Quaternion a, Quaternion b) {
    Quaternion res;
    res.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    res.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    res.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    res.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
    return res;
}

/**
 * @brief Division two quaternions
 * 
 * @param a Dividend quaternions
 * @param b Divisor quaternion 
 * @return result Quaternion 
 */
Quaternion quaternion_div(Quaternion a, Quaternion b) {
    Quaternion res;
    res.w = (b.w * a.w + b.x * a.x + b.y * a.y + b.z * a.z) / (b.w * b.w + b.x * b.x + b.y * b.y + b.z * b.z);
    res.x = (b.w * a.x - b.x * a.w - b.y * a.z + b.z * a.y) / (b.w * b.w + b.x * b.x + b.y * b.y + b.z * b.z);
    res.y = (b.w * a.y + b.x * a.z - b.y * a.w - b.z * a.x) / (b.w * b.w + b.x * b.x + b.y * b.y + b.z * b.z);
    res.z = (b.w * a.z - b.x * a.y + b.y * a.x - b.z * a.w) / (b.w * b.w + b.x * b.x + b.y * b.y + b.z * b.z);
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


/**
 * @brief Calculating Euler angles from quaternion
 * 
 * @param q Exist quaternion
 * @return struct EulerAngles 
 */
EulerAngles quaternion_to_euler(Quaternion q) {
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = copysign(3.141592653589793238463 / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = atan2(siny_cosp, cosy_cosp);

    angles.yaw *= 180.0/3.141592653589793238463;
    angles.pitch *= 180.0/3.141592653589793238463;
    angles.roll *= 180.0/3.141592653589793238463;

    return angles;

}
