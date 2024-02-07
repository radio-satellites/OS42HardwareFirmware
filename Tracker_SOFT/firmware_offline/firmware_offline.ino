#include "esp_camera.h"
#include "Arduino.h"
#include "FS.h"                
#include "SD_MMC.h"            
#include "soc/soc.h"           
#include "soc/rtc_cntl_reg.h"  
#include "driver/rtc_io.h"
//#include "horus_l2.h"
//#include "util.h"

#include <EEPROM.h>   
#include <RadioLib.h>

#include <SPI.h>

#define NSS_LORATX 12                                  //select on LoRa device
#define NRESET_LORATX 15                               //reset pin on LoRa device
#define SCK_LORATX 4                                   //SCK on SPI3
#define MISO_LORATX 13                                 //MISO on SPI3 
#define MOSI_LORATX 2                                  //MOSI on SPI3
        

// define the number of bytes you want to access
#define EEPROM_SIZE 1

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

int pictureNumber = 0;

//RADIO SETTINGS//

#define TX_FREQ         434.200
#define FSK4_BAUD       100
#define FSK4_SPACING    244

//Radio stuff
SPIClass spi(HSPI);
SPISettings spiSettings(2000000, MSBFIRST, SPI_MODE0);
SX1278 radio = new Module(NSS_LORATX, RADIOLIB_NC, NRESET_LORATX, RADIOLIB_NC, spi, spiSettings);

FSK4Client fsk4(&radio);

//HORUS BINARY V1 STRUCTURE

/*
// Horus Binary Packet Structure - Version 1
struct HorusBinaryPacketV1
{
    uint8_t     PayloadID;
    uint16_t  Counter;
    uint8_t Hours;
    uint8_t Minutes;
    uint8_t Seconds;
    float Latitude;
    float Longitude;
    uint16_t    Altitude;
    uint8_t     Speed;       // Speed in Knots (1-255 knots)
    uint8_t     Sats;
    int8_t      Temp;        // Twos Complement Temp value.
    uint8_t     BattVoltage; // 0 = 0.5v, 255 = 5.0V, linear steps in-between.
    uint16_t    Checksum;    // CRC16-CCITT Checksum.
}  __attribute__ ((packed));

char rawbuffer [128];   // Buffer to temporarily store a raw binary packet.
char codedbuffer [128]; // Buffer to store an encoded binary packet
uint16_t packet_count = 1;  // Packet counter

int build_horus_binary_packet(char *buffer){

  struct HorusBinaryPacketV1 BinaryPacket;

  BinaryPacket.PayloadID = 0; // 0 = 4FSKTEST. Refer https://github.com/projecthorus/horusdemodlib/blob/master/payload_id_list.txt
  BinaryPacket.Counter = packet_count;
  BinaryPacket.Hours = 12;
  BinaryPacket.Minutes = 34;
  BinaryPacket.Seconds = 56;
  BinaryPacket.Latitude = 0.0;
  BinaryPacket.Longitude = 0.0;
  BinaryPacket.Altitude = 0;
  BinaryPacket.Speed = 0;
  BinaryPacket.BattVoltage = 0;
  BinaryPacket.Sats = 0;
  BinaryPacket.Temp = 0;

  BinaryPacket.Checksum = (uint16_t)crc16((unsigned char*)&BinaryPacket,sizeof(BinaryPacket)-2);

  memcpy(buffer, &BinaryPacket, sizeof(BinaryPacket));
  
  return sizeof(struct HorusBinaryPacketV1);
}

*/

//Instead, use the example generated:

int horusPacketLen = 45;
byte horusPacket[] = {
  0x45, 0x24, 0x24, 0x48, 0x2F, 0x12, 0x16, 0x08, 0x15, 0xC1,
  0x49, 0xB2, 0x06, 0xFC, 0x92, 0xEB, 0x93, 0xD7, 0xEE, 0x5D,
  0x35, 0xA0, 0x91, 0xDA, 0x8D, 0x5F, 0x85, 0x6B, 0x63, 0x03,
  0x6B, 0x60, 0xEA, 0xFE, 0x55, 0x9D, 0xF1, 0xAB, 0xE5, 0x5E,
  0xDB, 0x7C, 0xDB, 0x21, 0x5A, 0x19
};

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
 
  Serial.begin(115200);
  
  camera_config_t config;
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
  config.pixel_format = PIXFORMAT_JPEG; 
  
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA; // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  
  // Init Camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  
  //Serial.println("Starting SD Card");
  if(!SD_MMC.begin()){
    Serial.println("SD Card Mount Failed");
    return;
  }
  
  uint8_t cardType = SD_MMC.cardType();
  if(cardType == CARD_NONE){
    Serial.println("No SD Card attached");
    return;
  }

  Serial.println("Radio start...");
  
  spi.begin(SCK_LORATX, MISO_LORATX, MOSI_LORATX, NSS_LORATX);
  
  int state = radio.beginFSK();

  if(state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while(true);
  }

  radio.setDataShaping(RADIOLIB_SHAPING_0_3);
  radio.setOutputPower(2.0);

  state = fsk4.begin(TX_FREQ, FSK4_SPACING, FSK4_BAUD);
  if(state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while(true);
  }

}

void loop() {

  fsk4.write(horusPacket, horusPacketLen);
  
  camera_fb_t * fb = NULL;
  
  // Take Picture with Camera
  fb = esp_camera_fb_get();  
  if(!fb) {
    Serial.println("Camera capture failed");
    return;
  }
  // initialize EEPROM with predefined size
  EEPROM.begin(EEPROM_SIZE);
  pictureNumber = EEPROM.read(0) + 1;

  // Path where new picture will be saved in SD Card
  String path = "/picture" + String(pictureNumber) +".jpg";

  fs::FS &fs = SD_MMC; 
  Serial.printf("Picture file name: %s\n", path.c_str());
  
  File file = fs.open(path.c_str(), FILE_WRITE);
  if(!file){
    Serial.println("Failed to open file in writing mode");
  } 
  else {
    file.write(fb->buf, fb->len); // payload (image), payload length
    Serial.printf("Saved file to path: %s\n", path.c_str());
    EEPROM.write(0, pictureNumber);
    EEPROM.commit();
  }
  file.close();
  esp_camera_fb_return(fb); 
}
