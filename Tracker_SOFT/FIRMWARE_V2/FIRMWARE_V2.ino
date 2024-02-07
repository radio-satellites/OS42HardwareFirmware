/* This is the image transmission firmware for OS-42
 * Written by Sasha VE3SVF
 * 
 *  SSDV from https://github.com/mkshrps/espcam/blob/master/src/main.cpp
 * PROTOCOL DESCRIPTION:
 * 
 * Sync word: 0x1ACFFC1D (4 bytes)
 * VCID: (1 byte)
 * 
 * VCID TABLE
 * | 0 | JPEG imagery data |
 * | 9 |   GPS information |
 * | 7 |      Filler       |
 * 
 * EEPROM ADDR MAP:
 * 
 * 60 - Global EEPROM Flash flag (e.g is there anything for us to read in the flash?)
 * 61 - ImageID variable storage
 * 62 - Global Software Fault storage (0-255). Each time we restart due to a software fault, this increases by one
 * 
 */

#include <RadioLib.h>
#include <SPI.h>
#include <EEPROM.h>
#include <TinyGPSPlus.h>

#include "esp_camera.h"
#include "esp_task_wdt.h"
#include "camera.h"
#include "util.h"
#include "ssdv.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"


camera_config_t config;

#define NSS_LORATX 12                                  //select on LoRa device
#define NRESET_LORATX 15                               //reset pin on LoRa device
#define SCK_LORATX 4                                   //SCK on SPI3
#define MISO_LORATX 13                                 //MISO on SPI3 
#define MOSI_LORATX 2                                  //MOSI on SPI3

#define uS_TO_S_FACTOR 1000000
#define TIME_TO_SLEEP  3 
#define EEPROM_SIZE 64

////////////////////////CHANGE THIS!!!////////////////////////
#define PAYLOAD_ID "T" //Single character
#define TEMP_CALIB 9 //Calibration factor for temperature sensor
#define IMG_INTERLEAVE 4 //Send an image every x boot times
#define RADIO_LEVEL 17.0 //Radio transmitter power level in dbM
#define RADIO_USERFO false //Used for lower power levels
#define WDT_PANIC 5 //WDT trigger time
#define USE_WDT true
//////////////////////////////////////////////////////////////

//Change these!

// ssdv definitions
#define JPEG_RESOLUTION             4                                // 0-8 corresponfing to 320x240, 352x288, 640x480, 800x480, 1024x768, 1280x960, 1600x1200, 2048x1536, 2592x1944
#define JPEG_QUALITY                3                                // 0-16 corresponding to 96.7, 93.7, 87.3, 81.2, 74.8, 68.6, 62.3, 56.2, 50.0, 44.4, 39.9, 36.3, 33.2, 30.7, 28.5, 26.6, 25.8
#define SSDV_QUALITY                7                                // 0-7 corresponding to JPEG quality: 13, 18, 29, 43, 50, 71, 86 and 100
#define IMG_BUFF_SIZE               128                               // size of the buffer feeding SSDV process 
#define FSK_BUFFER_LEN 255 //FSK buffer length


static const char callsign[] = "OS42-T";                    // maximum of 6 characters
uint8_t jpgRes                     = JPEG_RESOLUTION;
uint8_t jpgQuality                 = JPEG_QUALITY;
uint32_t imgSize                   = 0;
uint32_t imgCount                  = 1;
uint8_t ssdvQuality                = SSDV_QUALITY;
uint8_t imgBuff[IMG_BUFF_SIZE];
ssdv_t ssdv;
uint8_t fskBuff[FSK_BUFFER_LEN];

RTC_DATA_ATTR int imageID = 0;
RTC_DATA_ATTR int frameID = 0;
RTC_DATA_ATTR int softwareFaultCount = 0; //Used to determine whether or not we have a software fault
RTC_DATA_ATTR int bootCount = 0;

bool txSSDV = false;

bool readFromFlash = false;

SPIClass spi(HSPI);
SPISettings spiSettings(2000000, MSBFIRST, SPI_MODE0);
SX1278 radio = new Module(NSS_LORATX, RADIOLIB_NC, NRESET_LORATX, RADIOLIB_NC, spi, spiSettings);

TinyGPSPlus gps;

unsigned long previousMicros = micros(); //Timer in uS. 

int transmit_wait_factor = 1000; //Number of uS to wait during tx

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
bool read_GPS = true; //Start by reading the GPS first SET TO FALSE TO SKIP GPS FUNCTION. FOR TESTING ONLY!

#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif

/*

void setFlag(void) {
  // we sent a packet, set the flag
  transmittedFlag = true;
}

*/

bool configInitCamera()
{
  //Serial.println(F("Initialising the camera module "));

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;      //YUV422,GRAYSCALE,RGB565,JPEG

  //Select lower framesize if the camera doesn't support PSRAM
  if (psramFound())
  {
    //Serial.println(F("PSRAM found"));
    config.frame_size = FRAMESIZE_VGA;      //FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA, XUGA == 100K+, SVGA = 25K+ // QVGA = 320x240, VGA = 640x480, SVGA = 800x600 (Not div 16!), XGA = 1024x768, SXGA = 1280x1024, UXGA = 1600x1200
    config.jpeg_quality = 10;                //10-63 lower number means higher quality
    config.fb_count = 2;
  }
  else
  {
    //Serial.println(F("No PSRAM"));
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);   //Initialize the Camera
  if (err != ESP_OK)
  {
    Serial.printf("Camera init failed with error 0x%x", err);
    Serial.println();
    return false;
  }

  sensor_t * s = esp_camera_sensor_get();
  s->set_brightness(s, -2);     // -2 to 2
  s->set_contrast(s, 0);       // -2 to 2
  s->set_saturation(s, 0);     // -2 to 2
  s->set_special_effect(s, 0); // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
  s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
  s->set_awb_gain(s, 0);       // 0 = disable , 1 = enable
  s->set_wb_mode(s, 2);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
  s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable
  s->set_aec2(s, 1);           // 0 = disable , 1 = enable
  s->set_ae_level(s, 0);       // -2 to 2
  s->set_aec_value(s, 1200);    // 0 to 1200
  s->set_gain_ctrl(s, 1);      // 0 = disable , 1 = enable
  s->set_agc_gain(s, 0);       // 0 to 30
  s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6
  s->set_bpc(s, 1);            // 0 = disable , 1 = enable
  s->set_wpc(s, 0);            // 0 = disable , 1 = enable
  s->set_raw_gma(s, 1);        // 0 = disable , 1 = enable
  s->set_lenc(s, 0);           // 0 = disable , 1 = enable
  s->set_hmirror(s, 0);        // 0 = disable , 1 = enable
  s->set_vflip(s, 0);          // 0 = disable , 1 = enable
  s->set_dcw(s, 1);            // 0 = disable , 1 = enable
  s->set_colorbar(s, 0);       // 0 = disable , 1 = enable
  return true;
}

int iread(uint8_t *buffer,int numBytes,camera_fb_t *fb, int fbIndex ){
  int bufSize = 0;
  // have we reached past end of imagebuffer
  if((fbIndex + numBytes ) < fb->len) {
    bufSize = numBytes;
  }  else  {
    bufSize = fb->len - fbIndex;
  }
  // clear the dest buffer
  memset(buffer,0,numBytes);
  memcpy(buffer,&fb->buf[fbIndex],bufSize);
  return bufSize;
}


int process_ssdv(camera_fb_t *fb){
  int index = 0;
  int c = 0;
  int ssdvPacketCount = 0;

  ssdv_enc_init(&ssdv, SSDV_TYPE_NORMAL, (char *)callsign, imageID++, ssdvQuality, FSK_BUFFER_LEN);
  ssdv_enc_set_buffer(&ssdv, fskBuff);

  while (1){ //This ONLY exits when SSDV is finished or an SSDV send error occurs
    while((c = ssdv_enc_get_packet(&ssdv)) == SSDV_FEED_ME) {
        index += iread(imgBuff, IMG_BUFF_SIZE, fb, index);
        ssdv_enc_feed(&ssdv, imgBuff, IMG_BUFF_SIZE);
  }
    if(c == SSDV_EOI) {
      //Serial.print("SSDV said EOI");
      break; // Exit loop
    }
    else if(c != SSDV_OK){
      break; //Return back
    }
    //TX command goes here:
    
    radio.transmit(fskBuff,FSK_BUFFER_LEN);
    
    if (USE_WDT){
      esp_task_wdt_reset();
    }
    
    //radio.transmit("Hi from OS-42!");
    //printarrayHEX(fskBuff,0,sizeof(fskBuff));
    ssdvPacketCount++;
  }
  return ssdvPacketCount;
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
  dtostrf(latitude_OS, 10, 7, lat_string); //dtostrf(float_value, min_width, num_digits_after_decimal, where_to_store_string)
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

void setup() {
  //-------------Hardware Safety Checks here-------------
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 1); //enable brownout detector for boot
  if (USE_WDT){
    esp_task_wdt_init(WDT_PANIC, true); //enable panic so ESP32 restarts
    esp_task_wdt_add(NULL); //add current thread to WDT watch
  }
  //-----------------------------------------------------
  
  // put your setup code here, to run once:
  Serial.begin(9600); //This is to configure our GPS

  bootCount++;

  if (bootCount % IMG_INTERLEAVE == 0){
    txSSDV = true;
  }
  else{
    txSSDV = false;
  }

  // initialize SX1278 FSK modem with default settings
  //Serial.print(F("[SX1278] Initializing ... "));
  spi.begin(SCK_LORATX, MISO_LORATX, MOSI_LORATX, NSS_LORATX);
  
  //int state = radio.begin(434.0,125.0,7,RADIOLIB_SX127X_SYNC_WORD,2,8,0); //Init LoRa float freq=434.0, float bw=125.0, uint8_t sf=9, uint8_t cr=7, uint8_t syncWord=RADIOLIB_SX127X_SYNC_WORD, int8_t power=10, uint16_t preambleLength=8, uint8_t gain=0
  int state = radio.begin();
  
  radio.setFrequency(434.0);
  radio.setSpreadingFactor(7);
  radio.setBandwidth(500.0);
  radio.setCodingRate(8);

  //Set power

  radio.setOutputPower(RADIO_LEVEL,RADIO_USERFO);
  
  if (state == RADIOLIB_ERR_NONE) {
    //Serial.println(F("success!"));
  } else {
    //Serial.print(F("failed, code "));
    //Serial.println(state);
    while (true);
  }

  if (txSSDV){
    //Initiate camera
    bool cameraOK = configInitCamera();
  
    if (!cameraOK){
      //Yikes, camera does not work. Save progress and restart the CPU
      //EEPROM.write(61,1);
      //EEPROM.write(60,imageID);
      if (softwareFaultCount++ == 255){
        //Prevent overflow
        softwareFaultCount = 255;
        
      }
      /*
      EEPROM.write(62,softwareFaultCount);
      Serial.print("Wrote EEPROM: ");
      Serial.print(EEPROM.read(62));
      */
      ESP.restart();
      while(true) {}
    }
    
  }
  
 

  //Get GPS

  while(read_GPS){
   while (Serial.available() > 0){
    if (USE_WDT){
      esp_task_wdt_reset();
    }
    if (gps.encode(Serial.read())){
      if (gps.location.isValid()){
        radio.transmit("GPSLCK11111");
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

  //Reset for next time
  read_GPS = true;

  if (txSSDV){
     //Take picture

    camera_fb_t * fb = NULL;
    
    fb = esp_camera_fb_get();  
  
    //Send picture
  
    //Disable brownout detector
  
    //WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  
    process_ssdv(fb);
  
    //WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 1); //Enable it again

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
  
  //Turn off transmitter

  radio.reset();

  //Now, sleep
  sleepNow();
}

void sleepNow(){
  if (USE_WDT){
    esp_task_wdt_reset();
  }
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  esp_deep_sleep_start();
}

void loop() {
  //Nothing is called here.
}
