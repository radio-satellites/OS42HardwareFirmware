#include "GPSconfig.h"
#include <WiFi.h>

RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR int tx_APRS = 0;

void setup() {
  Serial.begin(9600);
  WiFi.setSleep(true); //We don't need WiFi now. 
  
  SendConfig(Set4G,sizeof(Set4G)); //Send 4G config
  
}

void loop() {
  
}
