const PROGMEM  uint8_t ClearConfig[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x01, 0x19, 0x98};
const PROGMEM  uint8_t GPGLLOff[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B};
const PROGMEM  uint8_t GPGSVOff[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39};
const PROGMEM  uint8_t GPVTGOff[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x47};
const PROGMEM  uint8_t GPGSAOff[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32};
const PROGMEM  uint8_t GPGGAOff[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x24};
const PROGMEM  uint8_t GPRMCOff[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x40};
const PROGMEM  uint8_t Navrate10hz[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12};
const PROGMEM  uint8_t LowPower[] = {0xb5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0x22, 0x92};
const PROGMEM  uint8_t SetRegular[] = {0xb5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x00, 0x21, 0x91};
const PROGMEM  uint8_t Set2G[] = {0xb5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xff, 0xff, 0x07, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xfa, 0x00, 0xfa, 0x00, 0x64, 0x00, 0x2c, 0x01, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4e, 0xfd};
const PROGMEM  uint8_t Set4G[] = {0xb5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xff, 0xff, 0x08, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xfa, 0x00, 0xfa, 0x00, 0x64, 0x00, 0x2c, 0x01, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4f, 0x1f};
const PROGMEM  uint8_t Set1G[] = {0xb5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xff, 0xff, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xfa, 0x00, 0xfa, 0x00, 0x64, 0x00, 0x2c, 0x01, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4d, 0xdb};
const PROGMEM  uint8_t GNSSOff[] = {0xb5, 0x62, 0x06, 0x57, 0x08, 0x00, 0x01, 0x00, 0x00, 0x00, 0x50, 0x4f, 0x54, 0x53, 0xac, 0x85};
const PROGMEM  uint8_t GNSSOn[] = {0xb5, 0x62, 0x06, 0x57, 0x08, 0x00, 0x01, 0x00, 0x00, 0x00, 0x20, 0x4e, 0x55, 0x52, 0x7b, 0xc3};
const PROGMEM  uint8_t SetSoftwareBackup[] = {0xb5, 0x62, 0x06, 0x57, 0x08, 0x00, 0x01, 0x00, 0x00, 0x00, 0x50, 0x4b, 0x43, 0x42, 0x86, 0x46};

void SendConfig(const uint8_t *Progmem_ptr, uint8_t arraysize) //Syntax: GPS object to send to, command array, command array size
{
  uint8_t byteread, index;

  Progmem_ptr = Progmem_ptr - arraysize;
  for (index = 0; index < arraysize; index++)
  {
    byteread = pgm_read_byte_near(Progmem_ptr++);
    Serial.write(byteread);
  }
  delay(50);

}