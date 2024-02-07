void printarrayHEX(uint8_t *buff, uint32_t startaddr, uint32_t len)
{
  uint32_t index;
  uint8_t buffdata;

  for (index = startaddr; index < (startaddr + len); index++)
  {
    buffdata = buff[index];
    if (buffdata < 16)
    {
      Serial.print(F("0"));
    }
    Serial.print(buffdata, HEX);
    Serial.print(F(" "));
  }
}
