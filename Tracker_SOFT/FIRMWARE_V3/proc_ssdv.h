uint8_t jpgRes                     = JPEG_RESOLUTION;
uint8_t jpgQuality                 = JPEG_QUALITY;
uint32_t imgSize                   = 0;
uint32_t imgCount                  = 1;
uint8_t ssdvQuality                = SSDV_QUALITY;
uint8_t imgBuff[IMG_BUFF_SIZE];
ssdv_t ssdv;
uint8_t fskBuff[FSK_BUFFER_LEN];

RTC_DATA_ATTR int imageID = 0;
RTC_DATA_ATTR int bootCount = 0;

void handleTemperature(){
  temperature = radio.getTempRaw()+TEMP_CALIB;
}

int process_ssdv(camera_fb_t *fb){
  int index = 0;
  int c = 0;
  int ssdvPacketCount = 0;

  if (USE_FEC == true){
    ssdv_enc_init(&ssdv, SSDV_TYPE_NORMAL, (char *)callsign, imageID++, ssdvQuality, FSK_BUFFER_LEN); //The reason we use SSDV_TYPE_NORMAL (e.g no FEC) here is because the RX software decodes with soft symbols
  }
  else{
    ssdv_enc_init(&ssdv, SSDV_TYPE_NOFEC, (char *)callsign, imageID++, ssdvQuality, FSK_BUFFER_LEN);
  }

  ssdv_enc_set_buffer(&ssdv, fskBuff);

  while (1){ //This ONLY exits when SSDV is finished or an SSDV send error occurs
    checkGPS(); //Empty GPS serial buffer
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
    if (ssdvPacketCount % GPS_INTERLEAVE == 0){
      //Update our GPS, then send a GPS packet
      checkGPS();
      sendGPSPacket();
    }
    ssdvPacketCount++;
  }
  //Send one temperature packet
  handleTemperature();
  sendTemperature();
  
  return ssdvPacketCount;
}

void sleepNow(){
  if (USE_WDT){
    esp_task_wdt_reset();
  }
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  esp_deep_sleep_start();
}
void sleepNow_LPM(){
  if (USE_WDT){
    esp_task_wdt_reset();
  }
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP_LPM * uS_TO_S_FACTOR);
  esp_deep_sleep_start();
}
