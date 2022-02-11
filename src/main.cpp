#include <Arduino.h>
#include <Wire.h>
#include "BNO055.h"
#include "SpaceHandler.h"

#define ADDRESS_0x28 0X28  //I2C address selection pin LOW
#define ADDRESS_0x29 0x29  //                          HIGH

#define DIRECTION_X -1
#define DIRECTION_Y -1
#define DIRECTION_Z -1


BNO055 root_sensor(ADDRESS_0x28);
BNO055 shoulder_sensor(ADDRESS_0x29);

Quaternion root_q;
Quaternion shoulder_q;
Quaternion final_q;
Angles final_angles;


void setup() {
  Wire.begin();
  Serial.begin(9600);
  root_sensor.init();
  shoulder_sensor.init();
}

void loop() {
  root_sensor.readQuat();
  shoulder_sensor.readQuat();

  root_q = create_quaternion_from_exist(
    root_sensor.quat.q1, // x
    root_sensor.quat.q2, // y
    root_sensor.quat.q3, // z
    root_sensor.quat.q0  // w
    );

  shoulder_q = create_quaternion_from_exist(
    shoulder_sensor.quat.q1, // x
    shoulder_sensor.quat.q2, // y
    shoulder_sensor.quat.q3, // z
    shoulder_sensor.quat.q0  // w
    );


  final_q = quaternion_mult(shoulder_q, quaternion_invert(root_q));
  final_angles = get_angles_from_quat(final_q, DIRECTION_X, DIRECTION_Y, DIRECTION_Z);

  Serial.printf("%f  %f  %f", final_angles.from_x, final_angles.from_y, final_angles.from_z);
}