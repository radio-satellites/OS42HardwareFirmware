/* OS-42 Firmware                                                        */
/*=======================================================================*/
/* Copyright 2023-2024 Sasha Timokhov <VE3SVF@gmail.com>                 */
/*                                                                       */
/* This program is free software: you can redistribute it and/or modify  */
/* it under the terms of the GNU General Public License as published by  */
/* the Free Software Foundation, either version 3 of the License or      */
/* any later version.                                                    */
/*                                                                       */
/* This program is distributed in the hope that it will be useful,       */
/* but WITHOUT ANY WARRANTY; without even the implied warranty of        */
/* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         */
/* GNU General Public License for more details.                          */
/*                                                                       */
/* You should have received a copy of the GNU General Public License     */
/* along with this program.  If not, see <http://www.gnu.org/licenses/>. */

#include <RadioLib.h>
#include <SPI.h>
#include <WiFi.h>
#include <TinyGPSPlus.h>

//Include header files

#include "esp_camera.h"
#include "esp_task_wdt.h"
#include "esp_system.h"
#include "definitions_config.h"
#include "camera.h"
#include "ssdv.h"
#include "GPS.h"
#include "proc_ssdv.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

unsigned long previousMicros = micros(); //Timer in uS.

RTC_DATA_ATTR int LPM_rotations = 0;

int transmit_wait_factor = 1000; //Number of uS to wait during tx

#if defined(ESP8266) || defined(ESP32)
ICACHE_RAM_ATTR
#endif


void setup() {
  //-------------Hardware Safety Checks here-------------
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 1); //enable brownout detector for boot
  if (USE_WDT) {
    esp_task_wdt_init(WDT_PANIC, true); //enable panic so ESP32 restarts
    esp_task_wdt_add(NULL); //add current thread to WDT watch
  }
  //-----------------------------------------------------
  Serial.begin(115200); //This is to configure our GPS

  bootCount++;

  WiFi.setSleep(true);

  spi.begin(SCK_LORATX, MISO_LORATX, MOSI_LORATX, NSS_LORATX);
  int state = radio.begin();

  radio.setFrequency(434.0);
  radio.setSpreadingFactor(7);
  radio.setBandwidth(500.0);
  radio.setCodingRate(8);
  radio.setOutputPower(RADIO_LEVEL, RADIO_USERFO);

  if (state == RADIOLIB_ERR_NONE) {
    if (DEBUG_MSG) {
      Serial.println(F("success!"));
    }
  } else {
    if (DEBUG_MSG) {
      Serial.print(F("failed, code "));
      Serial.println(state);
      while (true);
    }
  }

  if (txSSDV) {
    //Initiate camera
    bool cameraOK = configInitCamera();

    if (!cameraOK) { //If we are not OK, restart our entire system
      if (DEBUG_MSG) {
        Serial.print("Camera NOT OK!");
      }
      if (strcmp(CAM_ERROR_DO, "RESTART") == 0) {
        digitalWrite(PWDN_GPIO_NUM, HIGH); //Turn camera off
        delay(100000); //Instead of restarting, let the watchdog do stuff
        //ESP.restart();
        //while(true) {}
      }
      if (strcmp(CAM_ERROR_DO, "TXGPS+RESTART") == 0) {
        for (int i = 0; i < 20; i++) {
          checkGPS();
          sendGPSPacket();
          delay(30);
        }
        digitalWrite(PWDN_GPIO_NUM, HIGH); //Turn camera off
        delay(100000); //Instead of restarting, let the watchdog do stuff
        //ESP.restart();
        while (true) {}
      }
      if (strcmp(CAM_ERROR_DO, "TXGPS") == 0) {
        for (int i = 0; i < 20; i++) {
          checkGPS();
          sendGPSPacket();
          delay(30);
        }
      }

    }
    else {
      if (DEBUG_MSG) {
        Serial.print("Camera OK!");
      }
    }
  }

  if (SET_FLIGHT_MODE_YES) {
    Serial.end();
    Serial.begin(9600);
    if (DEBUG_MSG){
      Serial.print("Waiting for GPS boot...");
    }
    delay(DELAY_FLIGHT_MODE * 1000);
    esp_task_wdt_reset();
    setGPS_AirBorne();
    esp_task_wdt_reset();
    delay(200);
    setGPS_AirBorne();
    esp_task_wdt_reset();
    delay(200);
    setGPS_AirBorne();
    esp_task_wdt_reset();
  }

}

void loop() {
  if (LPM_rotations <= LPM_ITERATIONS){
    
  
  if (txSSDV) {
    //Take picture and send
    camera_fb_t * fb = NULL;
    fb = esp_camera_fb_get();
    if (DEBUG_MSG) {
      Serial.print("Got picture...");
    }
    process_ssdv(fb);
    esp_camera_fb_return(fb); //Free frame buffer
  }
  else {
    //Send a bunch of GPS telemetry packets
    radio.setSpreadingFactor(7);
    radio.setBandwidth(125.0);
    radio.setCodingRate(8);

    for (int i = 0; i < 10; i++) {
      checkGPS();
      sendGPSPacket();
      esp_task_wdt_reset();
    }
  }
  if (ALLOW_SLEEP) {
    //Turn off transmitter and sleep
    if (DEBUG_MSG) {
      Serial.print("Going to sleep!");
    }
    radio.reset();
    sleepNow();
  }
  else {
    if (DEBUG_MSG) {
      Serial.print("Delaying!");
    }
    for (int i = 0; i < TIME_TO_SLEEP * 10; i++) {
      delay(100);
      if (USE_WDT) {
        esp_task_wdt_reset();
      }
      checkGPS();
      sendGPSPacket();
    }

  }
  }
  else{
    //Low power mode is on!
    //First, tx some GPS packets
    LPM_rotations++;
    for (int i = 0; i < TIME_TO_SLEEP * 10; i++) {
      delay(1000);
      if (USE_WDT) {
        esp_task_wdt_reset();
      }
      checkGPS();
      sendGPSPacket();
    }
    //Now, we might occasionally send an image
    if (LPM_rotations % 5 == 0){
      if (txSSDV) {
        camera_fb_t * fb = NULL;
        fb = esp_camera_fb_get();
        if (DEBUG_MSG) {
          Serial.print("Got picture...");
        }
        process_ssdv(fb);
        esp_camera_fb_return(fb); //Free frame buffer
      }
    }
    sleepNow_LPM(); //Now, let's sleep. LPM_rotations is stored in RTC memory. 
  }
  LPM_rotations++;
}
