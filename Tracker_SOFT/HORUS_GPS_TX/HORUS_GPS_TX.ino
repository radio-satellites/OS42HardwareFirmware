#include <TinyGPSPlus.h>
#include <RadioLib.h>
#include <SPI.h>

#include "horus_l2.h"
//#include "mfsk.h"
#include "util.h"

#define NSS_LORATX 12                                  //select on LoRa device
#define NRESET_LORATX 15                               //reset pin on LoRa device
#define SCK_LORATX 4                                   //SCK on SPI3
#define MISO_LORATX 13                                 //MISO on SPI3 
#define MOSI_LORATX 2                                  //MOSI on SPI3

////////////////////////CHANGE THIS!!!////////////////////////
#define PAYLOAD_ID 0
#define TEMP_CALIB 9 //Calibration factor for temperature sensor
//////////////////////////////////////////////////////////////

SPIClass spi(HSPI);
SPISettings spiSettings(2000000, MSBFIRST, SPI_MODE0);
SX1278 radio = new Module(NSS_LORATX, RADIOLIB_NC, NRESET_LORATX, RADIOLIB_NC, spi, spiSettings);
FSK4Client fsk4(&radio);

TinyGPSPlus gps;

float latitude_OS = 0.0;
float longitude_OS = 0.0;
uint16_t altitude_OS = 0;
uint8_t speed_OS = 0;
uint8_t sats_OS = 0;

uint8_t time_hour = 0;
uint8_t time_minute = 0;
uint8_t time_second = 0;

bool altitude_valid = false;
bool speed_valid = false;
bool sats_valid = false;
bool time_valid = false;


bool read_GPS = true;

char rawbuffer [128];   // Buffer to temporarily store a raw binary packet.
char codedbuffer [128]; // Buffer to store an encoded binary packet
uint16_t packet_count = 1;  // Packet counter

int8_t temperature = 0;


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

int build_horus_binary_packet_v1(char *buffer){
  // Generate a Horus Binary v1 packet, and populate it with data.
  // The assignments in this function should be replaced with real data

  struct HorusBinaryPacketV1 BinaryPacket;

  BinaryPacket.PayloadID = PAYLOAD_ID; // 0 = 4FSKTEST. Refer https://github.com/projecthorus/horusdemodlib/blob/master/payload_id_list.txt
  BinaryPacket.Counter = packet_count;

  if (time_valid){
    BinaryPacket.Hours = time_hour;
    BinaryPacket.Minutes = time_minute;
    BinaryPacket.Seconds = time_second;
  }
  else{
    BinaryPacket.Hours = 0;
    BinaryPacket.Minutes = 0;
    BinaryPacket.Seconds = 0;
  }
  
  
  BinaryPacket.Latitude = latitude_OS;
  BinaryPacket.Longitude = longitude_OS; //No need to check these, as when the function is called the GPS is already good
  
  if (altitude_valid){
    BinaryPacket.Altitude = altitude_OS;
  }
  else{
    BinaryPacket.Altitude = 0;
  }
  
  if (speed_valid){
    BinaryPacket.Speed = speed_OS;
  }
  else{
    BinaryPacket.Speed = 0;
  }
  
  BinaryPacket.BattVoltage = 189; //Corresponds to 3.7v
  
  if (sats_valid){
    BinaryPacket.Sats = sats_OS;
  }
  else{
    BinaryPacket.Sats = 0;
  }
  
  BinaryPacket.Temp = temperature;

  BinaryPacket.Checksum = (uint16_t)new_crc16((unsigned char*)&BinaryPacket,sizeof(BinaryPacket)-2);

  memcpy(buffer, &BinaryPacket, sizeof(BinaryPacket));
  
  return sizeof(struct HorusBinaryPacketV1);
}


void handleGPS(){
  int pkt_len = build_horus_binary_packet_v1(rawbuffer);
  int coded_len = horus_l2_encode_tx_packet((unsigned char*)codedbuffer,(unsigned char*)rawbuffer,pkt_len);
  

  
  fsk4.write(27); //Preamble
  fsk4.write((uint8_t*)codedbuffer,coded_len);
  //fsk4.write((uint8_t*)b,sizeof(b));

  //fsk4_preamble(&radio, 8);
 // fsk4_write(&radio, codedbuffer, coded_len);
  
  packet_count++;
}

void handleTemperature(){
  temperature = radio.getTempRaw()+TEMP_CALIB;
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  spi.begin(SCK_LORATX, MISO_LORATX, MOSI_LORATX, NSS_LORATX);
  radio.beginFSK();

  radio.setDataShaping(RADIOLIB_SHAPING_0_3);
  radio.setOutputPower(2.0);

  // low ("space") frequency:     434.0 MHz
  // frequency shift:             244 Hz
  // baud rate:                   100 baud
  //fsk4_setup(&radio, 434.0, 244, 100);
  fsk4.begin(434.0, 244, 100);
  
  //fsk4.idle();
  
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
      if (gps.altitude.isValid()){
        altitude_OS = gps.altitude.meters();
        altitude_valid = true;
      }

      if (gps.speed.isValid()){
        speed_OS = round(gps.speed.kmph()*0.5399568); //Convert km/h to knots
        speed_valid = true;
      }

      if (gps.satellites.isValid()){
        sats_OS = gps.satellites.value();
        sats_valid = true;
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
  
  handleTemperature();
  
  handleGPS();

  
}
