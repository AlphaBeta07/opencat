#include <Arduino.h>

#include "Robot.h"
#include "config.h"
#include "FoxWiFi.h"

#if ENABLE_WEBAPI
#include "WebAPI.h"
#endif

Robot robot;

void setup() {
  Serial.begin(115200);
  Serial.println("");
  Serial.println("Starting...");

  robot.init();
  delay(1000);

  setupWiFi();
  setupOTA();
  
  #if ENABLE_WEBAPI
    setupWebServer();
  #endif
}

void loop() {
  handleOTA();
  
  #if ENABLE_WEBAPI
    server.handleClient();
  #endif
  
  robot.update();
}