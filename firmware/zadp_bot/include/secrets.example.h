// secrets.example.h : copy this file to include/secrets.h and fill in your values.
// include/secrets.h is git-ignored (firmware/.gitignore): never commit it.
#pragma once

#define WIFI_SSID "your-network-name"
#define WIFI_PASS "your-network-password"
#define MQTT_HOST "192.168.1.10"     // IP of the PC running the Mosquitto broker (= P.mqtt.broker in twin_params.m)

// Optional, only if the broker requires a login:
// #define MQTT_USER "zadp"
// #define MQTT_PASS "secret"
