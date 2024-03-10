////////////////////////CHANGE THESE////////////////////////
//Change these for a regular flight. These are the "basic" settings
static const char callsign[] = "OS42-1"; // maximum of 6 characters. "T" should be used for testing, "X" for unknown and any other letter of the alphabet to differentiate between flights
#define TEMP_CALIB 2 //Calibration factor for temperature sensor. Calibrate your own sensor. See calibration.txt.
#define GPS_INTERLEAVE 3 //Send a GPS packet every X imagery packets
#define IMAGERY_START_ITERATIONS  5 //After X initial GPS telemetry cycles, start sending imagery
#define LPM_ITERATIONS 200 //After X images, we go into low power mode

//Radio power levels
//RADIO_USERFO true power levels:
//-3 to +15dbM
//RADIO_USERFO false power levels
//+2 to +17dbM. High power operation (+20dbM) is also partially supported)

#define RADIO_LEVEL 17.0 //Radio transmitter power level in dBm
#define RADIO_USERFO false //Used for lower power levels. 

#define txSSDV true //Enable/disable SSDV functionality. If off, we will send a lot of GPS telemetry packets
#define TYPE_GPS "10S" //What GPS is hooked up to the port? "6M" supports all legacy NEO-6M GPS modules. "10S" supports MAX-10S and clones. NOTE: NEO-6M is less stable and you might need to restart many times to get the nav mode on. 
////////////////////////////////////////////////////////////

////////////////////////ADVANCED SETTINGS////////////////////////
#define USE_WDT true //Enable/disable watchdog for testing
#define DEBUG_MSG false //Allow debug messages through the Serial port
#define ALLOW_SLEEP false //Do we restart between images? Usually it's not a good idea, unless we have memory issues
#define CAM_ERROR_DO "TXGPS+RESTART" //What do we do when there is a camera error? Options: RESTART,TXGPS+RESTART,TXGPS
#define SET_FLIGHT_MODE_YES true //Turn on flight mode?
#define DELAY_FLIGHT_MODE 2 //Time to wait for the GPS to boot (only used if above is on)
#define TIME_TO_SLEEP 3 //Time between cycles
#define TIME_TO_SLEEP_LPM 6
#define WDT_PANIC 5 //WDT trigger time in seconds
#define SSDV_QUALITY                7                                //0-7 corresponding to JPEG quality: 13, 18, 29, 43, 50, 71, 86 and 100
#define IMG_BUFF_SIZE               128                               //size of the buffer feeding SSDV encoder
#define FSK_BUFFER_LEN 255 //FSK buffer length
#define USE_FEC true //Encode FEC as part of the SSDV packet
/////////////////////////////////////////////////////////////////

//Do not change
#define NSS_LORATX 12                                  //select on LoRa device
#define NRESET_LORATX 15                               //R
#define SCK_LORATX 4                                   //SCK on SPI3
#define MISO_LORATX 13                                 //MISO on SPI3 
#define MOSI_LORATX 2                                  //MOSI on SPI3

#define uS_TO_S_FACTOR 1000000 //Change this if you want to change TIME ITSELF
