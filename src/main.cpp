#include <Arduino.h>
#include <Wire.h>
#include "BNO055.h"
#include "SpaceHandler.h"
#include "MQTT.h"
#include "config.h"
#include "ArduinoJson.h"
#include "ADS1X15.h"

// #define EULER_ANGELS

#define ADDRESS_SHOULDER 0X28   // I2C address selection pin LOW
#define ADDRESS_ROOT 0x29       // HIGH

#define DIRECTION_X -1
#define DIRECTION_Y -1
#define DIRECTION_Z -1

BNO055 shoulder_sensor(ADDRESS_SHOULDER);
BNO055 root_sensor(ADDRESS_ROOT);

Quaternion root_quat;
Quaternion shoulder_quat;
Quaternion final_quat;
Angles final_angles;
#ifdef EULER_ANGELS
EulerAngles final_euler_angles;
#endif

String data;

MQTTController mqtt;

Adafruit_ADS1115 elbow_sensor;

void callback(char *topic, byte *payload, unsigned int length);

void setup() {

#ifdef DEBUG
  Serial.begin(9600);
#endif

  Wire.begin();
  root_sensor.init();
  shoulder_sensor.init();

  pinMode(ELBOW_SENSOR_PIN, INPUT);

  mqtt.initialize(
    MQTT_WIFI_SSID,
    MQTT_WIFI_PASSWORD,
    MQTT_SERVER,
    MQTT_PORT,
    MQTT_USER,
    MQTT_PASSWORD,
    MQTT_CLIENT_ID);

  if (!elbow_sensor.begin(0x48))
    Serial.println("Failed to initialize elbow sensor");
}

void loop() {
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
  json_obj["from_x"] = final_angles.from_x;
  json_obj["from_y"] = final_angles.from_y;
  json_obj["from_z"] = final_angles.from_z;
  json_obj["elbow"] = elbow_sensor.readADC_SingleEnded(0);
  serializeJson(json_obj, json_arr);

  // mqtt.send(MQTT_ARMS_TOPIC, json_arr);

#ifdef DEBUG
#ifndef EULER_ANGELS
  // data  = String(root_q.x) + "  " + String(root_q.y) + "  " + String(root_q.z) + "  " + String(root_q.w);
  data = String(final_angles.from_x) + "  " + String(final_angles.from_y) + "  " + String(final_angles.from_z);
#else
  data = String(final_angles.from_x) + "  " + String(final_angles.from_y) + "  " + String(final_angles.from_z)
       + "  |  " + String(final_euler_angles.yaw) + "  " + String(final_euler_angles.pitch) + "  " + String(final_euler_angles.roll);
#endif
  Serial.println(json_arr);
#endif

  delay(DELAY_MS);
}