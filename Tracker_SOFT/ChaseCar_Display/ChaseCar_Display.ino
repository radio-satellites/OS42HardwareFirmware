#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x3F, 16, 2); // Set the LCD address to 0x27 for a 16x2 display

int counter = 0;
int line = 0;
void setup() {
  Serial.begin(115200); // Initialize serial communication at 9600 bits per second
  lcd.init();      // Initialize the LCD
  lcd.backlight(); // Turn on the backlight
  lcd.clear();     // Clear the LCD screen
  lcd.setCursor(0, 0);
}

void loop() {
  if (Serial.available() > 0) { // Check if data is available to read
    char data = Serial.read(); // Read the incoming data
    // Process the data
    //Serial.print("Received data: ");
    //Serial.println(data);
    if (data == '>') {
      line = 1;
      counter = 0;
    }
    else if (data == '<') {
      line = 0;
      counter = 0;
    }
    else if (data == '%'){
      lcd.clear();
    }
    else {
      if (sizeof(data) >= 1) {
        lcd.setCursor(counter, line);
        lcd.print(data);
        counter++;
      }
    }
  }
}
