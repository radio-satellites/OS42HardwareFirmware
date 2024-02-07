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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  spi.begin(SCK_LORATX, MISO_LORATX, MOSI_LORATX, NSS_LORATX);
  
  int state = radio.beginFSK(434.0,1.2,10,250,2,256,false);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }
  radio.setOutputPower(-3,true); //Set to -3dBm by using the RFO pin instead of the PA_BOOST pin
  
}

void loop() {
  // put your main code here, to run repeatedly:
  //byte byteArr[] = {0xAB,0xAB,0xAB,0xAB};
  radio.transmit("Hello World!");
}
