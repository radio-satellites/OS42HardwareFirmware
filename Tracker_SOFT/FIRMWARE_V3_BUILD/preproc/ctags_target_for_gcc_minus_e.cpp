# 1 "C:\\Users\\Sasha\\Documents\\OS-42-tracker\\Tracker_SOFT\\FIRMWARE_V3\\FIRMWARE_V3.ino"
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

# 19 "C:\\Users\\Sasha\\Documents\\OS-42-tracker\\Tracker_SOFT\\FIRMWARE_V3\\FIRMWARE_V3.ino" 2
# 20 "C:\\Users\\Sasha\\Documents\\OS-42-tracker\\Tracker_SOFT\\FIRMWARE_V3\\FIRMWARE_V3.ino" 2
# 21 "C:\\Users\\Sasha\\Documents\\OS-42-tracker\\Tracker_SOFT\\FIRMWARE_V3\\FIRMWARE_V3.ino" 2

//Include header files

# 25 "C:\\Users\\Sasha\\Documents\\OS-42-tracker\\Tracker_SOFT\\FIRMWARE_V3\\FIRMWARE_V3.ino" 2
# 26 "C:\\Users\\Sasha\\Documents\\OS-42-tracker\\Tracker_SOFT\\FIRMWARE_V3\\FIRMWARE_V3.ino" 2
# 27 "C:\\Users\\Sasha\\Documents\\OS-42-tracker\\Tracker_SOFT\\FIRMWARE_V3\\FIRMWARE_V3.ino" 2
# 28 "C:\\Users\\Sasha\\Documents\\OS-42-tracker\\Tracker_SOFT\\FIRMWARE_V3\\FIRMWARE_V3.ino" 2
# 29 "C:\\Users\\Sasha\\Documents\\OS-42-tracker\\Tracker_SOFT\\FIRMWARE_V3\\FIRMWARE_V3.ino" 2
# 30 "C:\\Users\\Sasha\\Documents\\OS-42-tracker\\Tracker_SOFT\\FIRMWARE_V3\\FIRMWARE_V3.ino" 2
# 31 "C:\\Users\\Sasha\\Documents\\OS-42-tracker\\Tracker_SOFT\\FIRMWARE_V3\\FIRMWARE_V3.ino" 2

# 33 "C:\\Users\\Sasha\\Documents\\OS-42-tracker\\Tracker_SOFT\\FIRMWARE_V3\\FIRMWARE_V3.ino" 2

unsigned long previousMicros = micros(); //Timer in uS. 

int transmit_wait_factor = 1000; //Number of uS to wait during tx


 



void setup() {
  //-------------Hardware Safety Checks here-------------
  ({ do { if (__builtin_constant_p(!(((((0x3ff48000 + 0xd4))) >= 0x3ff00000) && (((0x3ff48000 + 0xd4))) <= 0x3ff13FFC)) && !(!(((((0x3ff48000 + 0xd4))) >= 0x3ff00000) && (((0x3ff48000 + 0xd4))) <= 0x3ff13FFC))) { extern __attribute__((error("(Cannot use WRITE_PERI_REG for DPORT registers use DPORT_WRITE_PERI_REG)"))) void failed_compile_time_assert(void); failed_compile_time_assert(); } 
# 45 "C:\\Users\\Sasha\\Documents\\OS-42-tracker\\Tracker_SOFT\\FIRMWARE_V3\\FIRMWARE_V3.ino" 3 4
 (__builtin_expect(!!(
# 45 "C:\\Users\\Sasha\\Documents\\OS-42-tracker\\Tracker_SOFT\\FIRMWARE_V3\\FIRMWARE_V3.ino"
 "(Cannot use WRITE_PERI_REG for DPORT registers use DPORT_WRITE_PERI_REG)" && (!(((((0x3ff48000 + 0xd4))) >= 0x3ff00000) && (((0x3ff48000 + 0xd4))) <= 0x3ff13FFC))
# 45 "C:\\Users\\Sasha\\Documents\\OS-42-tracker\\Tracker_SOFT\\FIRMWARE_V3\\FIRMWARE_V3.ino" 3 4
 ), 1) ? (void)0 : __assert_func ((__builtin_strrchr( "/" "C:\\Users\\Sasha\\Documents\\OS-42-tracker\\Tracker_SOFT\\FIRMWARE_V3\\FIRMWARE_V3.ino", '/') + 1), 45, __PRETTY_FUNCTION__, 
# 45 "C:\\Users\\Sasha\\Documents\\OS-42-tracker\\Tracker_SOFT\\FIRMWARE_V3\\FIRMWARE_V3.ino"
 "\"(Cannot use WRITE_PERI_REG for DPORT registers use DPORT_WRITE_PERI_REG)\" && (!(((((0x3ff48000 + 0xd4))) >= 0x3ff00000) && (((0x3ff48000 + 0xd4))) <= 0x3ff13FFC))"
# 45 "C:\\Users\\Sasha\\Documents\\OS-42-tracker\\Tracker_SOFT\\FIRMWARE_V3\\FIRMWARE_V3.ino" 3 4
 ))
# 45 "C:\\Users\\Sasha\\Documents\\OS-42-tracker\\Tracker_SOFT\\FIRMWARE_V3\\FIRMWARE_V3.ino"
 ; } while(0);; (*((volatile uint32_t *)((0x3ff48000 + 0xd4)))) = (uint32_t)(1); }); //enable brownout detector for boot
  if (true /*Enable/disable watchdog for testing*/){
    esp_task_wdt_init(5 /*WDT trigger time in seconds*/, true); //enable panic so ESP32 restarts
    esp_task_wdt_add(
# 48 "C:\\Users\\Sasha\\Documents\\OS-42-tracker\\Tracker_SOFT\\FIRMWARE_V3\\FIRMWARE_V3.ino" 3 4
                    __null
# 48 "C:\\Users\\Sasha\\Documents\\OS-42-tracker\\Tracker_SOFT\\FIRMWARE_V3\\FIRMWARE_V3.ino"
                        ); //add current thread to WDT watch
  }
  //-----------------------------------------------------
  Serial.begin(9600); //This is to configure our GPS

  //Configure some GPS stuff
  setGPS_AirBorne();

  bootCount++;

  spi.begin(4 /*SCK on SPI3*/, 13 /*MISO on SPI3 */, 2 /*MOSI on SPI3*/, 12 /*select on LoRa device*/);
  int state = radio.begin();

  radio.setFrequency(434.0);
  radio.setSpreadingFactor(7);
  radio.setBandwidth(500.0);
  radio.setCodingRate(8);
  radio.setOutputPower(17.0 /*Radio transmitter power level in dBm*/,false /*Used for lower power levels. */);

  if (state == (0)) {
    if (true /*Allow debug messages through the Serial port*/){
      Serial.println(((reinterpret_cast<const __FlashStringHelper *>(("success!")))));
    }
  } else {
    if (true /*Allow debug messages through the Serial port*/){
      Serial.print(((reinterpret_cast<const __FlashStringHelper *>(("failed, code ")))));
      Serial.println(state);
      while (true);
    }
  }

  if (true /*Enable/disable SSDV functionality. If off, we will send a lot of GPS telemetry packets*/){
    //Initiate camera
    bool cameraOK = configInitCamera();

    if (!cameraOK){ //If we are not OK, restart our entire system
      if (true /*Allow debug messages through the Serial port*/){
        Serial.print("Camera NOT OK!");
      }
      ESP.restart();
      while(true) {}
    }
    else{
      if (true /*Allow debug messages through the Serial port*/){
        Serial.print("Camera OK!");
      }
    }
  }

}

void loop() {
  if (true /*Enable/disable SSDV functionality. If off, we will send a lot of GPS telemetry packets*/){
     //Take picture and send
    camera_fb_t * fb = 
# 102 "C:\\Users\\Sasha\\Documents\\OS-42-tracker\\Tracker_SOFT\\FIRMWARE_V3\\FIRMWARE_V3.ino" 3 4
                      __null
# 102 "C:\\Users\\Sasha\\Documents\\OS-42-tracker\\Tracker_SOFT\\FIRMWARE_V3\\FIRMWARE_V3.ino"
                          ;
    fb = esp_camera_fb_get();
    if (true /*Allow debug messages through the Serial port*/){
      Serial.print("Got picture...");
    }
    process_ssdv(fb);
  }
  else{
    //Send a bunch of GPS telemetry packets
    radio.setSpreadingFactor(7);
    radio.setBandwidth(125.0);
    radio.setCodingRate(8);

    for (int i = 0; i < 10;i++){
      sendGPSPacket();
    }
  }
  if (false /*Do we restart between images? Usually it's not a good idea, unless we have memory issues*/){
  //Turn off transmitter and sleep
  radio.reset();
  sleepNow();
  }
  else{
    delay(3 /*Time between cycles*/*1000);
  }
}
