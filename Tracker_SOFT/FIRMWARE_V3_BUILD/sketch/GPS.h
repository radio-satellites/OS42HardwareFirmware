#line 1 "C:\\Users\\Sasha\\Documents\\OS-42-tracker\\Tracker_SOFT\\FIRMWARE_V3\\GPS.h"
/*
 * A lot of the brilliant UBX parsing code comes from KevWal's implementation here:
 * https://github.com/KevWal/ESP32-Cam-Pico-Project/blob/main/Code/ESP32-Cam-Pico-V3/ESP32-Cam-GPS.h
*/
SPIClass spi(HSPI);
SPISettings spiSettings(2000000, MSBFIRST, SPI_MODE0);
SX1278 radio = new Module(NSS_LORATX, RADIOLIB_NC, NRESET_LORATX, RADIOLIB_NC, spi, spiSettings);

TinyGPSPlus gps;

float latitude_OS = 0.0;
float longitude_OS = 0.0;
uint16_t altitude_OS = 0;
uint8_t speed_OS = 0;
uint8_t time_hour = 0;
uint8_t time_minute = 0;
uint8_t time_second = 0;

bool altitude_valid = false;
bool speed_valid = false;
bool time_valid = false;
bool read_GPS = true; //Start by reading the GPS first. Set to false to test other parts of the code and skip the GPS function entirely.

RTC_DATA_ATTR int frameID = 0;

int8_t temperature = 0;

int sendTemperature(){
  
  if (USE_WDT){
    esp_task_wdt_reset();
  }

  char temperature_string[20];
  char datastring[30];

  itoa(temperature,temperature_string,10);

  sprintf(datastring, "!!,%s",temperature_string);

  //Send buffer
  radio.transmit(datastring,sizeof(datastring));

  return 0;
}
  

int sendGPSPacket(){
  
  if (USE_WDT){
    esp_task_wdt_reset();
  }
  
  char datastring[255]; //GPS datastring, UKHAS-style
  char alt_string[20];
  char lat_string[20];
  char long_string[20];
  char frame_num_string[20];
  //char temp_string[20];

  //Convert latitude/longitude into char buffers
  dtostrf(latitude_OS, 10, 7, lat_string); 
  dtostrf(longitude_OS, 10, 7, long_string);

  //Convert frame id

  itoa(frameID,frame_num_string,10);

  //Convert altitude

  if (altitude_valid){
    dtostrf(altitude_OS, 7, 2, alt_string);
  }
  else{
    sprintf(alt_string,"UK");
  }

  ///Build buffer
  sprintf(datastring, "$$,%s,%s,%s,%s",alt_string,lat_string,long_string,frame_num_string);

  //Send buffer
  radio.transmit(datastring,sizeof(datastring));

  frameID++;
  
  return sizeof(datastring);
}

void checkGPS(){
  while (Serial.available() > 0){
    if (USE_WDT){
      esp_task_wdt_reset();
    }
    if (gps.encode(Serial.read())){
      if (gps.location.isValid()){
        latitude_OS = gps.location.lat();
        longitude_OS = gps.location.lng();
        read_GPS = false;
      }
      if (gps.altitude.isValid()){
        altitude_OS = gps.altitude.meters();
        altitude_valid = true;
      }

      if (gps.speed.isValid()){
        speed_OS = round(gps.speed.kmph()*0.5399568); //Convert km/h to knots
        speed_valid = true;
      }

      if (gps.satellites.isValid()){ //Satellites indicates we have lock, hence time
        time_hour = gps.time.hour();
        time_minute = gps.time.minute();
        time_second = gps.time.second();
        
        time_valid = true;
      }     
      
    }
  }
}


void sendUBX(const uint8_t *MSG, uint8_t len)
{
  
  Serial.flush();
  Serial.write(0xFF);
  delay(500);
  for (int i = 0; i < len; i++) {
    //Serial.write(MSG[i]);
    Serial.write(pgm_read_byte(MSG + i));
  }
}

boolean getUBX_ACK(const uint8_t *MSG)
{
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  uint32_t startTime = millis();

  // Construct the expected ACK packet
  ackPacket[0] = 0xB5; // header
  ackPacket[1] = 0x62; // header
  ackPacket[2] = 0x05; // class
  ackPacket[3] = 0x01; // id
  ackPacket[4] = 0x02; // length
  ackPacket[5] = 0x00;
  ackPacket[6] = pgm_read_byte(MSG + 2); // ACK class
  ackPacket[7] = pgm_read_byte(MSG + 3); // ACK id
  ackPacket[8] = 0; // CK_A
  ackPacket[9] = 0; // CK_B

  // Calculate the checksums
  for (uint8_t ubxi = 2; ubxi < 8; ubxi++) {
    ackPacket[8] = ackPacket[8] + ackPacket[ubxi];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }

  while (1) {
    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      return true;
    }
    if (millis() - startTime > 2000) {
      return false;
    }
    // Make sure data is available to read
    if (Serial.available()) {
      b = Serial.read();
      if (b == ackPacket[ackByteID]) {
        ackByteID++;
      }
      else {
        ackByteID = 0;
      }
    }
  }
}


void setGPS_AirBorne() // < 1g
{
  const static uint8_t PROGMEM setdm6[44] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06,
    0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
    0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC
  };
  sendUBX(setdm6, 44);
}

void gps_reset()
{
  const static uint8_t PROGMEM set_reset[12] = {0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0xFF, 0x87, 0x00, 0x00, 0x94, 0xF5};
  sendUBX(set_reset, 12);
  getUBX_ACK(set_reset);
}


void gps_set_max()
{
  const static uint8_t PROGMEM setMax[10] = {0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x00, 0x21, 0x91};
  sendUBX(setMax, 10);
  getUBX_ACK(setMax);
}

void gps_set_min()
{
  // Cyclic 1 second, from here: https://ukhas.org.uk/guides:ublox_psm
  const static uint8_t PROGMEM setPSM[10] = { 0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0x22, 0x92 };
  sendUBX(setPSM, 10);
  getUBX_ACK(setPSM);
}
