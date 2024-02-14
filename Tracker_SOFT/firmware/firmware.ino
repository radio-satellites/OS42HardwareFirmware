#include "GPSconfig.h"

void setup() {
  Serial.begin(9600);
  SendConfig(Set4G,sizeof(Set1G)); //Send 1G config
  
}

void loop() {
  
}
