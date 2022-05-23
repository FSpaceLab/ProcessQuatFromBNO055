#ifndef CONFIG_H
#define CONFIG_H

#define DEBUG  // TODO comment it

#ifdef DEBUG
#define LOG(str) Serial.println(str);
#else
#define LOG(str)
#endif

#define MQTT_WIFI_SSID "FSpace"
#define MQTT_WIFI_PASSWORD "freakspace#main"
#define MQTT_SERVER "teleoperate.annaone.com"
#define MQTT_PORT 1883
#define MQTT_USER "magic"
#define MQTT_PASSWORD "zEbl042_Chug"
#define MQTT_CLIENT_ID "smart_glove"
#define MQTT_ARMS_TOPIC "anna_1/arms"

#define DELAY_MS 70

//#define SHOULDER_SENSOR_ADDR 0X29   // chock
//#define ROOT_BACK_SENSOR_ADDR 0x28 // chock
#define SHOULDER_SENSOR_ADDR 0X28 // LOW
#define ROOT_BACK_SENSOR_ADDR 0x29 // HIGH
#define ELBOW_SENSOR_ADDR 0x48
#define WRIST_X_SENSOR_ADDR 0x8
#define WRIST_Y_SENSOR_ADDR 0x9
#define THUMB_FINGER_SENSOR_ADDR 0x10
#define INDEX_FINGER_SENSOR_ADDR 0x11
#define MIDDLE_FINGER_SENSOR_ADDR 0x12
#define RING_FINGER_SENSOR_ADDR 0x13
#define LITTLE_FINGER_SENSOR_ADDR 0x14

#endif