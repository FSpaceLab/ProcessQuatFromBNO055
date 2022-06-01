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
#define DIRECTION_X 1
#define DIRECTION_Y 1
#define DIRECTION_Z 1

BNO055 root_sensor;
BNO055 shoulder_sensor;
Adafruit_ADS1115 elbow_sensor;
ADS wrist_x_sensor;
ADS wrist_y_sensor;
ADS thumb_finger_sensor;
ADS index_finger_sensor;
ADS middle_finger_sensor;
ADS ring_finger_sensor;
ADS little_finger_sensor;

bool is_root_sensor_available = false;
bool is_shoulder_sensor_available = false;

Quaternion root_quat;
Quaternion shoulder_quat;
Quaternion final_quat;
Angles final_angles;

MQTTController mqtt;

// void callback(char *topic, byte *payload, unsigned int length);

void setup() 
{
#ifdef DEBUG
  Serial.begin(9600);
#endif

  while (!Wire.begin())
  {
    LOG("Failed to initialize I2C interface. Retrying...");
    delay(DELAY_MS);
    break; // TODO remove it
  }
  LOG("I2C interface initialized successfully");
  
  is_root_sensor_available = root_sensor.begin(ROOT_BACK_SENSOR_ADDR);
  if (is_root_sensor_available) {LOG("root_sensor initialized successfully");} else {LOG("Failed to initialize root_sensor");}
  LOG("Calibration of ROOT IMU");

  is_shoulder_sensor_available = shoulder_sensor.begin(SHOULDER_SENSOR_ADDR);
  if (is_shoulder_sensor_available) {LOG("shoulder_sensor initialized successfully");} else {LOG("Failed to initialize shoulder_sensor");}
  LOG("Calibration of SHOULDER IMU");

  while (!elbow_sensor.begin(ELBOW_SENSOR_ADDR))
  {
    LOG("Failed to initialize elbow_sensor. Retrying...");
    delay(DELAY_MS);
    break; // TODO remove it
  }
  LOG("elbow_sensor initialized successfully");
  
  while(!wrist_x_sensor.begin(WRIST_X_SENSOR_ADDR))
  {
    LOG("Failed to initialize wrist_x_sensor. Retrying...");
    delay(DELAY_MS);
    break; // TODO remove it
  }
  LOG("wrist_x_sensor initialized successfully");

  while(!wrist_y_sensor.begin(WRIST_Y_SENSOR_ADDR))
  {
    LOG("Failed to initialize wrist_y_sensor. Retrying...");
    delay(DELAY_MS);
    break; // TODO remove it
  }
  LOG("wrist_y_sensor initialized successfully");
  
  while(!thumb_finger_sensor.begin(THUMB_FINGER_SENSOR_ADDR))
  {
    LOG("Failed to initialize thumb_finger_sensor. Retrying...");
    delay(DELAY_MS);
    break; // TODO remove it
  }
  LOG("thumb_finger_sensor initialized successfully");

  while(!index_finger_sensor.begin(INDEX_FINGER_SENSOR_ADDR))
  {
    LOG("Failed to initialize index_finger_sensor. Retrying...");
    delay(DELAY_MS);
    break; // TODO remove it
  }
  LOG("index_finger_sensor initialized successfully");

  while(!middle_finger_sensor.begin(MIDDLE_FINGER_SENSOR_ADDR))
  {
    LOG("Failed to initialize middle_finger_sensor. Retrying...");
    delay(DELAY_MS);
    break; // TODO remove it
  }
  LOG("middle_finger_sensor initialized successfully");

  while(!ring_finger_sensor.begin(RING_FINGER_SENSOR_ADDR))
  {
    LOG("Failed to initialize ring_finger_sensor. Retrying...");
    delay(DELAY_MS);
    break; // TODO remove it
  }
  LOG("ring_finger_sensor initialized successfully");

  while(!little_finger_sensor.begin(LITTLE_FINGER_SENSOR_ADDR))
  {
    LOG("Failed to initialize little_finger_sensor. Retrying...");
    delay(DELAY_MS);
    break; // TODO remove it
  }
  LOG("little_finger_sensor initialized successfully");

  mqtt.initialize(
    MQTT_WIFI_SSID,
    MQTT_WIFI_PASSWORD,
    MQTT_SERVER,
    MQTT_PORT,
    MQTT_USER,
    MQTT_PASSWORD,
    MQTT_CLIENT_ID);
    

  #ifdef CALIBRATING_IMU_ENABLE
  // CALIBRATING OF THE IMU SENSORS
  StaticJsonDocument<256> calibrating_json_obj;
  char calibrating_json_arr[256];
  
  if (is_root_sensor_available) 
  {
    while (!root_sensor.isCalibrated())
    {
      calibrating_json_obj["name"] = "ROOT";
      calibrating_json_obj["Sys"] = String(root_sensor.getCalibrationSys(), DEC);
      calibrating_json_obj["Gyr"] = String(root_sensor.getCalibrationGyr(), DEC);
      calibrating_json_obj["Acc"] = String(root_sensor.getCalibrationAcc(), DEC);
      calibrating_json_obj["Mag"] = String(root_sensor.getCalibrationMag(), DEC);
      serializeJson(calibrating_json_obj, calibrating_json_arr);
      LOG(calibrating_json_arr);
      mqtt.send(MQTT_ARMS_TOPIC, calibrating_json_arr);
    }
  }

  if (is_shoulder_sensor_available) 
  {
    while (!shoulder_sensor.isCalibrated())
    {
      calibrating_json_obj["name"] = "SHOULDER";
      calibrating_json_obj["Sys"] = String(shoulder_sensor.getCalibrationSys(), DEC);
      calibrating_json_obj["Gyr"] = String(shoulder_sensor.getCalibrationGyr(), DEC);
      calibrating_json_obj["Acc"] = String(shoulder_sensor.getCalibrationAcc(), DEC);
      calibrating_json_obj["Mag"] = String(shoulder_sensor.getCalibrationMag(), DEC);
      serializeJson(calibrating_json_obj, calibrating_json_arr);
      LOG(calibrating_json_arr);
      mqtt.send(MQTT_ARMS_TOPIC, calibrating_json_arr);
    }
  }
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
  // final_quat = quaternion_mult(root_quat, shoulder_quat);
  // final_quat = quaternion_mult(shoulder_quat, root_quat);
  final_angles = get_angles_from_quat(final_quat, DIRECTION_X, DIRECTION_Y, DIRECTION_Z);

  StaticJsonDocument<256> json_obj;
  char json_arr[256];
  json_obj["sx"] = is_root_sensor_available && is_shoulder_sensor_available ? String(final_angles.from_z) : "-1";
  json_obj["sy"] = is_root_sensor_available && is_shoulder_sensor_available ? String(final_angles.from_x) : "-1";
  json_obj["sz"] = is_root_sensor_available && is_shoulder_sensor_available ? String(final_angles.from_y) : "-1";
  json_obj["el"] = elbow_sensor.isConnected() ? String(elbow_sensor.readADC_SingleEnded(0)) : "-1";
  json_obj["wx"] = wrist_x_sensor.isConnected() && wrist_x_sensor.available() ? String(wrist_x_sensor.getX()) : "-1";
  json_obj["wy"] = wrist_y_sensor.isConnected() && wrist_y_sensor.available() ? String(wrist_y_sensor.getX()) : "-1";
  json_obj["f0"] = thumb_finger_sensor.isConnected() && thumb_finger_sensor.available() ? String(thumb_finger_sensor.getX()) : "-1";
  json_obj["f1"] = index_finger_sensor.isConnected() && index_finger_sensor.available() ? String(index_finger_sensor.getX()) : "-1";
  json_obj["f2"] = middle_finger_sensor.isConnected() && middle_finger_sensor.available() ? String(middle_finger_sensor.getX()) : "-1";
  json_obj["f3"] = ring_finger_sensor.isConnected() && ring_finger_sensor.available() ? String(ring_finger_sensor.getX()) : "-1";
  json_obj["f4"] = little_finger_sensor.isConnected() && little_finger_sensor.available() ? String(little_finger_sensor.getX()) : "-1";
  serializeJson(json_obj, json_arr);
  
  mqtt.send(MQTT_ARMS_TOPIC, json_arr);

  // String data = String(root_quat.x) + "  " + String(root_quat.y) + "  " + String(root_quat.z) + "  " + String(root_quat.w); // TODO remove it
  // String data = String(shoulder_quat.x) + "  " + String(shoulder_quat.y) + "  " + String(shoulder_quat.z) + "  " + String(shoulder_quat.w); // TODO remove it
  // String data = String(final_angles.from_x) + "  " + String(final_angles.from_y) + "  " + String(final_angles.from_z); // TODO remove it
  LOG(json_arr);

  delay(DELAY_MS);
}