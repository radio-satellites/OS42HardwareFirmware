#include <TinyGPSPlus.h>
#include <RadioLib.h>
#include <SPI.h>

#define NSS_LORATX 12                                  //select on LoRa device
#define NRESET_LORATX 15                               //reset pin on LoRa device
#define SCK_LORATX 4                                   //SCK on SPI3
#define MISO_LORATX 13                                 //MISO on SPI3 
#define MOSI_LORATX 2                                  //MOSI on SPI3

SPIClass spi(HSPI);
SPISettings spiSettings(2000000, MSBFIRST, SPI_MODE0);
SX1278 radio = new Module(NSS_LORATX, RADIOLIB_NC, NRESET_LORATX, RADIOLIB_NC, spi, spiSettings);
RTTYClient rtty(&radio);

TinyGPSPlus gps;

float latitude_OS = 0.0;
float longitude_OS = 0.0;

bool read_GPS = true;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  spi.begin(SCK_LORATX, MISO_LORATX, MOSI_LORATX, NSS_LORATX);
  radio.beginFSK();

  // low ("space") frequency:     434.0 MHz
  // frequency shift:             183 Hz
  // baud rate:                   45 baud
  // encoding:                    ASCII (7-bit)
  // stop bits:                   1
  rtty.begin(434.0, 183, 45);
  rtty.idle();
  
}

void loop() {
  // put your main code here, to run repeatedly:
  while(read_GPS){
   while (Serial.available() > 0){
    if (gps.encode(Serial.read())){
      if (gps.location.isValid()){
        latitude_OS = gps.location.lat();
        longitude_OS = gps.location.lng();
        read_GPS = false;
      }
      
    }
   }
  }

  //Reset for next time
  read_GPS = true;

  rtty.println("OS-42:");
  rtty.println(latitude_OS,8);
  rtty.println(longitude_OS,8);

  
}
