void loop() {

  Serial.println("[RADIO] bitrate=200,deviation=300");

  //Take picture

  camera_fb_t * fb = NULL;
  
  fb = esp_camera_fb_get();  

  //Now, fb->buf is the buffer, fb->len is the length
  int image_size = fb->len;
  uint8_t*image_ptr = fb->buf;


  int repeat_large = floor(image_size/63);
  int number_small = image_size-(63*repeat_large);

  //Transmit the chunks that are larger than 63:
  
  for (int i = 0; i < repeat_large; i++) {
    uint8_t chunk[63];
    //Fill up buffer
    for (int x = 0; x < 63; x++){
      chunk[x] = *image_ptr;
      image_ptr++;
    }
    //while(!transmittedFlag){}
    //We transmitted!
    //transmittedFlag = false;
    //printarrayHEX(chunk,0,63); //Print hex array to serial: chunk, start address 0, length 63. 
    //radio.transmit(chunk,63); 
    
    bool startTransmitNow = false;
    while(!startTransmitNow){
      unsigned long currentMicros = micros();
      if (currentMicros - previousMicros >= transmit_wait_factor) { //Expected air time is 210uS, so we can use this to wait
        previousMicros = currentMicros;
        startTransmitNow = true;
      }
    }
    radio.startTransmit(chunk,63);
    
  }

   //Now, transmit the remainding chunk that is smaller than 63
  uint8_t chunk[63];

  for (int y = 0; y < number_small;y++){
   chunk[y] = *image_ptr;
   image_ptr++;
  }

  //radio.transmit(chunk,number_small);
  radio.transmit(chunk,number_small);
  //printarrayHEX(chunk,0,number_small);
   
   

  esp_camera_fb_return(fb); 
  
}

 /*
    bool startTransmitNow = false;
    while(!startTransmitNow){
      unsigned long currentMicros = micros();
      if (currentMicros - previousMicros >= transmit_wait_factor) { //Expected air time is 210uS, so we can use this to wait
        previousMicros = currentMicros;
        startTransmitNow = true;
      }
    }
    
    radio.startTransmit(fskBuff,62);

    */

    
    
    //radio.transmit(fskBuff,62);
    //Serial.println("Memory heap size: ");
    //Serial.print(esp_get_free_heap_size());
    //printarrayHEX(fskBuff,0,62);
