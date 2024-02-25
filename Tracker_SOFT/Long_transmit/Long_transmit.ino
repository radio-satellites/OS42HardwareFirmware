// include the library
#include <RadioLib.h>

#define NSS_LORATX 12                                  //select on LoRa device
#define NRESET_LORATX 15                               //reset pin on LoRa device
#define SCK_LORATX 4                                   //SCK on SPI3
#define MISO_LORATX 13                                 //MISO on SPI3 
#define MOSI_LORATX 2                                  //MOSI on SPI3

SPIClass spi(HSPI);
SPISettings spiSettings(2000000, MSBFIRST, SPI_MODE0);
SX1278 radio = new Module(NSS_LORATX, RADIOLIB_NC, NRESET_LORATX, RADIOLIB_NC, spi, spiSettings);

// save transmission state between loops
int transmissionState = RADIOLIB_ERR_NONE;

// this packet is much longer than would normally fit
// into SX1278's internal buffer
String longPacket = "Lorem ipsum dolor sit amet, consectetur adipiscing elit.\
 Maecenas at urna ut nunc imperdiet laoreet. Aliquam erat volutpat.\
 Etiam mattis mauris vitae posuere tincidunt. In sit amet bibendum nisl,\
 a ultrices lorem. Duis hendrerit ultricies condimentum. Phasellus eget nisi\
 eget massa aliquam bibendum. Pellentesque ante neque, aliquam non diam non,\
 fringilla facilisis ipsum. Morbi in molestie orci. Vestibulum luctus\
 venenatis arcu sit amet pellentesque. Nulla posuere sit amet turpis\
 id pharetra. Curabitur nec.";

void setup() {
  Serial.begin(9600);

  // initialize SX1278 with default settings
  Serial.print(F("[SX1278] Initializing ... "));
  spi.begin(SCK_LORATX, MISO_LORATX, MOSI_LORATX, NSS_LORATX);
  int state = radio.beginFSK(434.0,4.8,5.0);

  // when using one of the non-LoRa modules for Stream transmit
  // (RF69, CC1101, Si4432 etc.), use the basic begin() method
  // int state = radio.begin();

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }

  // set the function that will be called
  // when transmit buffer is empty
  radio.setFifoEmptyAction(fifoAdd);

  // fixed packet length mode is required
  radio.fixedPacketLengthMode(0);

  // start transmitting the long packet
  Serial.print(F("[SX1278] Sending a very long packet ... "));
  transmissionState = radio.startTransmit(longPacket);
}

// flag to indicate we can keep adding more bytes to FIFO
volatile bool fifoEmpty = false;

// flag to indicate that a packet was sent
bool transmittedFlag = false;

// how many bytes are there in total
int totalLength = longPacket.length();

// counter to keep track of how many bytes still need to be sent
int remLength = totalLength;

// this function is called when the radio transmit buffer
// is empty and ready to be refilled
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void fifoAdd(void) {
  // we can send more bytes
  fifoEmpty = true;
}

void loop() {
  if(fifoEmpty) {
    // reset flag
    fifoEmpty = false;

    // add more bytes to the transmit buffer
    uint8_t* txBuffPtr = (uint8_t*)longPacket.c_str();
    transmittedFlag = radio.fifoAdd(txBuffPtr, totalLength, &remLength);
  }

  Serial.println(remLength);

  // check if the previous transmission finished
  if(remLength == 0) {
    // reset flag
    transmittedFlag = false;

    // reset the counter
    remLength = totalLength;

    // NOTE: in FSK mode, SX127x will not automatically
    //       turn transmitter off after sending a packet
    //       set mode to standby to ensure we don't jam others
    radio.standby();

    // wait 10ms before transmitting again
    delay(100);

    // send another one
    Serial.print(F("[SX1278] Sending another long packet ... "));
    transmissionState = radio.startTransmit(longPacket);
  }
}
