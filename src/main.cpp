#include <Arduino.h>
#include <Wire.h>
#include "BNO055.h"
#include "SpaceHandler.h"
#include "MQTT.h"
#include "config.h"
#include "ArduinoJson.h"

#define ADDRESS_1 0X28  //I2C address selection pin LOW
#define ADDRESS_2 0x29  //                          HIGH

#define DIRECTION_X -1
#define DIRECTION_Y -1
#define DIRECTION_Z -1


BNO055 root_sensor(ADDRESS_2);
BNO055 shoulder_sensor(ADDRESS_1);

Quaternion root_q;
Quaternion shoulder_q;
Quaternion final_q;
Angles final_a;
Angles root_a;
EulerAngles final_ea;

String data;

MQTTController mqtt;

void callback(char *topic, byte *payload, unsigned int length);

void setup() {
  Wire.begin();
  Serial.begin(9600);
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


  // final_q = quaternion_mult(shoulder_q, quaternion_invert(root_q));
  // root_a = get_angles_from_quat(root_q, DIRECTION_X, DIRECTION_Y, DIRECTION_Z);

  final_q = quaternion_div(root_q, shoulder_q);
  final_a = get_angles_from_quat(final_q, DIRECTION_X, DIRECTION_Y, DIRECTION_Z);


  final_ea = quaternion_to_euler(final_q);

  StaticJsonDocument<256> doc;
  char out[128];

  doc["from_x"] = final_a.from_x;
  doc["from_y"] = final_a.from_y;
  doc["from_z"] = final_a.from_z;
  doc["elbow"] = analogRead(ELBOW_SENSOR_PIN);

  int b = serializeJson(doc, out);

  mqtt.send(MQTT_ARMS_TOPIC, out);
  
  data = String();

  // data  = String(root_q.x) + "  " + String(root_q.y) + "  " + String(root_q.z) + "  " + String(root_q.w);

  data = String(final_a.from_x) + "  " + String(final_a.from_y) + "  " + String(final_a.from_z) + "  |  " + String(final_ea.yaw) + "  " + String(final_ea.pitch) + "  " + String(final_ea.roll);
  // data = String(final_ea.yaw) + "  " + String(final_ea.pitch) + "  " + String(final_ea.roll);

  // data = String(final_q.w) + "  " + String(final_q.x) + "  " + String(final_q.y) + "  " + String(final_q.z);
  Serial.println(data);
  delay(70);
}