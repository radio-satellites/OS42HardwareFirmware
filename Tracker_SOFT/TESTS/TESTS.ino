#include "esp_sntp.h"
time_t now;
char strftime_buf[64];
struct tm timeinfo;


void setup() {
  // put your setup code here, to run once:
  time(&now);
// Set timezone to China Standard Time
  setenv("TZ", "CST-8", 1);
  tzset();
  
  localtime_r(&now, &timeinfo);
  strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
  
}

void loop() {
  // put your main code here, to run repeatedly:

}
