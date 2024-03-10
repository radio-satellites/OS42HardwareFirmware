/*
   A lot of the brilliant UBX parsing code comes from UKHAS's implementation here:
   https://ukhas.org.uk/doku.php?id=guides:ublox6
   Some of the other code comes from https://stuartsprojects.github.io/2018/08/26/Generating-UBLOX-GPS-Configuration-Messages.html

*/

//NEO-6M
const PROGMEM  uint8_t ClearConfig[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x01, 0x19, 0x98};
const PROGMEM  uint8_t GPGLLOff[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B};
const PROGMEM  uint8_t GPGSVOff[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39};
const PROGMEM  uint8_t GPVTGOff[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x47};
const PROGMEM  uint8_t GPGSAOff[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32};
const PROGMEM  uint8_t GPGGAOff[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x24};
const PROGMEM  uint8_t GPRMCOff[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x40};
const PROGMEM  uint8_t Navrate10hz[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12};
const PROGMEM  uint8_t LowPower[] = {0xb5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0x22, 0x92};
const PROGMEM  uint8_t SetRegular[] = {0xb5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x00, 0x21, 0x91};
const PROGMEM  uint8_t Set2G[] = {0xb5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xff, 0xff, 0x07, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xfa, 0x00, 0xfa, 0x00, 0x64, 0x00, 0x2c, 0x01, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4e, 0xfd};
const PROGMEM  uint8_t Set4G[] = {0xb5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xff, 0xff, 0x08, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xfa, 0x00, 0xfa, 0x00, 0x64, 0x00, 0x2c, 0x01, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4f, 0x1f};
const PROGMEM  uint8_t Set1G[] = {0xb5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xff, 0xff, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xfa, 0x00, 0xfa, 0x00, 0x64, 0x00, 0x2c, 0x01, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4d, 0xdb};
const PROGMEM  uint8_t GNSSOff[] = {0xb5, 0x62, 0x06, 0x57, 0x08, 0x00, 0x01, 0x00, 0x00, 0x00, 0x50, 0x4f, 0x54, 0x53, 0xac, 0x85};
const PROGMEM  uint8_t GNSSOn[] = {0xb5, 0x62, 0x06, 0x57, 0x08, 0x00, 0x01, 0x00, 0x00, 0x00, 0x20, 0x4e, 0x55, 0x52, 0x7b, 0xc3};
const PROGMEM  uint8_t SetSoftwareBackup[] = {0xb5, 0x62, 0x06, 0x57, 0x08, 0x00, 0x01, 0x00, 0x00, 0x00, 0x50, 0x4b, 0x43, 0x42, 0x86, 0x46};


//MAX-10S

uint8_t Set1G_10S[] = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x01, 0x01, 0x00, 0x00, 0x21, 0x00, 0x11, 0x20, 0x06, 0xF3, 0x58};
uint8_t SetPassive_10S[] = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x01, 0x01, 0x00, 0x00, 0x10, 0x00, 0x41, 0x20, 0x01, 0x0D, 0x8E};
byte gps_set_success = 0;


SPIClass spi(HSPI);
SPISettings spiSettings(2000000, MSBFIRST, SPI_MODE0);
SX1278 radio = new Module(NSS_LORATX, RADIOLIB_NC, NRESET_LORATX, RADIOLIB_NC, spi, spiSettings);

TinyGPSPlus gps;

float latitude_OS = 0.0;
float longitude_OS = 0.0;
uint32_t altitude_OS = 0;
int16_t speed_OS = 0;
uint8_t time_hour = 0;
uint8_t time_minute = 0;
uint8_t time_second = 0;

bool altitude_valid = false;
bool speed_valid = false;
bool time_valid = false;
bool loc_valid = false;

RTC_DATA_ATTR int frameID = 0;

int8_t temperature = 0;



int sendTemperature() {

  if (USE_WDT) {
    esp_task_wdt_reset();
  }

  char temperature_string[20];
  char datastring[30];

  itoa(temperature, temperature_string, 10);

  sprintf(datastring, "!!,%s", temperature_string);

  //Send buffer
  radio.transmit(datastring, sizeof(datastring));

  return 0;
}


int sendGPSPacket() {

  if (USE_WDT) {
    esp_task_wdt_reset();
  }

  char datastring[255]; //GPS datastring, UKHAS-style
  char alt_string[20];
  char lat_string[20];
  char long_string[20];
  char frame_num_string[20];
  char speed_string[20];
  char hour_string[10];
  char minute_string[10];
  char second_string[10];

  //Convert latitude/longitude into char buffers
  dtostrf(latitude_OS, 10, 7, lat_string);
  dtostrf(longitude_OS, 10, 7, long_string);

  //Convert frame id

  itoa(frameID, frame_num_string, 10);

  //Convert altitude

  if (altitude_valid) {
    dtostrf(altitude_OS, 7, 2, alt_string);
  }
  else {
    sprintf(alt_string, "UK");
  }

  if (speed_valid) {
    dtostrf(speed_OS, 7, 2, speed_string);
  }
  else {
    sprintf(speed_string, "UK");
  }

  if (time_valid) {
    dtostrf(time_hour, 2, 0, hour_string);
    dtostrf(time_minute, 2, 0, minute_string);
    dtostrf(time_second, 2, 0, second_string);
  }
  else {
    sprintf(hour_string, "UK");
    sprintf(minute_string, "UK");
    sprintf(second_string, "UK");
  }

  ///Build buffer
  sprintf(datastring, "$$,%s,%s,%s,%s,%s,%s,%s,%s", alt_string, lat_string, long_string, speed_string, hour_string, minute_string, second_string, frame_num_string);

  //Send buffer
  radio.transmit(datastring, sizeof(datastring));

  frameID++;

  return sizeof(datastring);
}

void checkGPS() {
  while (Serial.available() > 0) {
    if (USE_WDT) {
      esp_task_wdt_reset();
    }
    if (gps.encode(Serial.read())) {
      if (gps.location.isValid()) {
        latitude_OS = gps.location.lat();
        longitude_OS = gps.location.lng();
        loc_valid = true;
      }
      if (gps.altitude.isValid()) {
        altitude_OS = gps.altitude.meters();
        altitude_valid = true;
      }

      if (gps.speed.isValid()) {
        speed_OS = gps.speed.mps(); //Speed in m/s
        speed_valid = true;
      }

      if (gps.satellites.isValid()) { //Satellites indicates we have lock, hence time
        time_hour = gps.time.hour();
        time_minute = gps.time.minute();
        time_second = gps.time.second();

        time_valid = true;
      }

    }
  }
}

void sendUBX(uint8_t *MSG, uint8_t len) {
  for (int i = 0; i < len; i++) {
    Serial.write(MSG[i]);
    //mySerial.print(MSG[i], HEX);
  }
  Serial.println();
}
boolean getUBX_ACK(uint8_t *MSG) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();
  //Serial.print(" * Reading ACK response: ");

  // Construct the expected ACK packet
  ackPacket[0] = 0xB5;  // header
  ackPacket[1] = 0x62;  // header
  ackPacket[2] = 0x05;  // class
  ackPacket[3] = 0x01;  // id
  ackPacket[4] = 0x02;  // length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];  // ACK class
  ackPacket[7] = MSG[3];  // ACK id
  ackPacket[8] = 0;   // CK_A
  ackPacket[9] = 0;   // CK_B

  // Calculate the checksums
  for (uint8_t i = 2; i < 8; i++) {
    ackPacket[8] = ackPacket[8] + ackPacket[i];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }

  while (1) {

    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      if (DEBUG_MSG) {
        Serial.println(" (SUCCESS!)");
      }
      return true;
    }

    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) {
      if (DEBUG_MSG) {
        Serial.println(" (FAILED!)");
      }
      return false;
    }

    // Make sure data is available to read
    if (Serial.available()) {
      b = Serial.read();

      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) {
        ackByteID++;
        //mySerial.print(b, HEX);
      }
      else {
        ackByteID = 0;  // Reset and look again, invalid order
      }

    }
  }
}

void SendConfig(const uint8_t *Progmem_ptr, uint8_t arraysize) //Syntax: GPS object to send to, command array, command array size
{
  uint8_t byteread, index;

  Progmem_ptr = Progmem_ptr - arraysize;
  for (index = 0; index < arraysize; index++)
  {
    byteread = pgm_read_byte_near(Progmem_ptr++);
    Serial.write(byteread);
  }
  delay(50);

}


void setGPS_AirBorne_10S(){
  //Don't send any Serial before
  while(!gps_set_success){
    delay(200);
    sendUBX(Set1G_10S,17); //Send 1G config
    gps_set_success = getUBX_ACK(Set1G_10S);
  }

  //Just set passive mode as well
  sendUBX(SetPassive_10S,17);
}

void setGPS_AirBorne_6M() { //Sets NEO-6M GPS to 1g airborne mode
  if (DEBUG_MSG) {
    Serial.print("About to send NEO-6M GPS config!");

  }
  esp_task_wdt_reset();
  SendConfig(Set1G, sizeof(Set1G));
  if (DEBUG_MSG) {
    Serial.print("Sent!");
  }
  esp_task_wdt_reset();
}
