#include <Arduino.h>
#include <Wire.h>
#include "BNO055.h"
#include "SpaceHandler.h"
#include "MQTT.h"
#include "config.h"
#include "ArduinoJson.h"
#include "ADS1X15.h"
#include "SparkFunFlexes.h" 

// #define EULER_ANGELS

#define ADDRESS_SHOULDER 0X28   // I2C address selection pin LOW
#define ADDRESS_ROOT 0x29       // HIGH

#define DIRECTION_X -1
#define DIRECTION_Y -1
#define DIRECTION_Z -1

BNO055 shoulder_sensor(ADDRESS_SHOULDER);
BNO055 root_sensor(ADDRESS_ROOT);

Adafruit_ADS1115 elbow_sensor;

ADS wrist_x_sensor;
ADS wrist_y_sensor;

ADS thumb_finger_sensor;
ADS index_finger_sensor;
ADS middle_finger_sensor;
ADS ring_finger_sensor;
ADS little_finger_sensor;

Quaternion root_quat;
Quaternion shoulder_quat;
Quaternion final_quat;
Angles final_angles;
#ifdef EULER_ANGELS
EulerAngles final_euler_angles;
#endif

MQTTController mqtt;

String data;

// void callback(char *topic, byte *payload, unsigned int length);

void setup() 
{
  Wire.begin();
  while (!Wire.begin())
  {
    LOG("Setup failed to initialize I2C interface. Retrying...");
    delay(DELAY_MS);
  }

  root_sensor.init();
  shoulder_sensor.init();

  while (!elbow_sensor.begin(ELBOW_SENSOR_ADDR))
  {
    LOG("Setup failed to initialize elbow_sensor. Retrying...");
    delay(DELAY_MS);
  }
  while(!wrist_x_sensor.begin(WRIST_X_SENSOR_ADDR))
  {
    LOG("Setup failed to initialize wrist_x_sensor. Retrying...");
    delay(DELAY_MS);
  }
  while(!wrist_y_sensor.begin(WRIST_Y_SENSOR_ADDR))
  {
    LOG("Setup failed to initialize wrist_y_sensor. Retrying...");
    delay(DELAY_MS);
  }
  while(!thumb_finger_sensor.begin(THUMB_FINGER_SENSOR_ADDR))
  {
    LOG("Setup failed to initialize thumb_finger_sensor. Retrying...");
    delay(DELAY_MS);
  }
  while(!index_finger_sensor.begin(INDEX_FINGER_SENSOR_ADDR))
  {
    LOG("Setup failed to initialize index_finger_sensor. Retrying...");
    delay(DELAY_MS);
  }
  while(!middle_finger_sensor.begin(MIDDLE_FINGER_SENSOR_ADDR))
  {
    LOG("Setup failed to initialize middle_finger_sensor. Retrying...");
    delay(DELAY_MS);
  }
  while(!ring_finger_sensor.begin(RING_FINGER_SENSOR_ADDR))
  {
    LOG("Setup failed to initialize ring_finger_sensor. Retrying...");
    delay(DELAY_MS);
  }
  while(!little_finger_sensor.begin(LITTLE_FINGER_SENSOR_ADDR))
  {
    LOG("Setup failed to initialize little_finger_sensor. Retrying...");
    delay(DELAY_MS);
  }

  mqtt.initialize(
    MQTT_WIFI_SSID,
    MQTT_WIFI_PASSWORD,
    MQTT_SERVER,
    MQTT_PORT,
    MQTT_USER,
    MQTT_PASSWORD,
    MQTT_CLIENT_ID);

#ifdef DEBUG
  Serial.begin(9600);
#endif
}

void loop() 
{
  root_sensor.readQuat();
  shoulder_sensor.readQuat();

  root_quat = create_quaternion_from_exist(
    root_sensor.quat.q1, // x
    root_sensor.quat.q2, // y
    root_sensor.quat.q3, // z
    root_sensor.quat.q0  // w
    );

  shoulder_quat = create_quaternion_from_exist(
    shoulder_sensor.quat.q1, // x
    shoulder_sensor.quat.q2, // y
    shoulder_sensor.quat.q3, // z
    shoulder_sensor.quat.q0  // w
    );

  final_quat = quaternion_div(root_quat, shoulder_quat);
  final_angles = get_angles_from_quat(final_quat, DIRECTION_X, DIRECTION_Y, DIRECTION_Z);

#ifdef EULER_ANGELS
  final_euler_angles = quaternion_to_euler(final_quat);
#endif

  StaticJsonDocument<256> json_obj;
  char json_arr[128];
  json_obj["shoulder_x"] = final_angles.from_x;
  json_obj["shoulder_y"] = final_angles.from_y;
  json_obj["shoulder_z"] = final_angles.from_z;
  json_obj["elbow"] = elbow_sensor.readADC_SingleEnded(0);
  json_obj["wrist_x"] = wrist_x_sensor.available() ? String(wrist_x_sensor.getX()) : "-1";
  json_obj["wrist_y"] = wrist_y_sensor.available() ? String(wrist_y_sensor.getX()) : "-1";
  json_obj["thumb_finger"] = thumb_finger_sensor.available() ? String(thumb_finger_sensor.getX()) : "-1";
  json_obj["index_finger"] = index_finger_sensor.available() ? String(index_finger_sensor.getX()) : "-1";
  json_obj["middle_finger"] = middle_finger_sensor.available() ? String(middle_finger_sensor.getX()) : "-1";
  json_obj["ring_finger"] = ring_finger_sensor.available() ? String(ring_finger_sensor.getX()) : "-1";
  json_obj["little_finger"] = little_finger_sensor.available() ? String(little_finger_sensor.getX()) : "-1";
  serializeJson(json_obj, json_arr);

  // mqtt.send(MQTT_ARMS_TOPIC, json_arr);

#ifdef DEBUG
#ifndef EULER_ANGELS
  // data  = String(root_q.x) + "  " + String(root_q.y) + "  " + String(root_q.z) + "  " + String(root_q.w);
  // data = String(final_angles.from_x) + "  " + String(final_angles.from_y) + "  " + String(final_angles.from_z);
#else
  data = String(final_angles.from_x) + "  " + String(final_angles.from_y) + "  " + String(final_angles.from_z)
       + "  |  " + String(final_euler_angles.yaw) + "  " + String(final_euler_angles.pitch) + "  " + String(final_euler_angles.roll);
#endif
  LOG(json_arr);
#endif

  delay(DELAY_MS);
}