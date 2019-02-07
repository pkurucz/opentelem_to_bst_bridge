/*
 * This is a Arduino project that handles converting
 * Eagletree Vector Open Telemetry serial data (57600 Baud)
 * to Team Blacksheep (TBS) BST (Blacksheep Telemetry)
 * on an I2C bus.
 */


#include "vector_open_telemetry.h"
#include "bst_telemetry.h"
#include <Wire.h>

/* ----------------------------------------------------- */

#define LED_ON() digitalWrite(LED_BUILTIN, HIGH)
#define LED_OFF() digitalWrite(LED_BUILTIN, LOW)

/* ----------------------------------------------------- */

// the setup routine runs once when you press reset:
void setup() {
  Serial.begin(115200, SERIAL_8N1);

  /* I2C Init, make sure the internal pull-ups are enabled */
#if 1
  pinMode (SDA, INPUT_PULLUP);
  pinMode (SCL, INPUT_PULLUP);
  digitalWrite(SDA, 1);
  digitalWrite(SCL, 1);
#endif
  Wire.begin();
  
  pinMode(LED_BUILTIN, OUTPUT);
  LED_OFF();

  while (!Serial);             // Leonardo: wait for serial monitor
  Serial.println("\nI2C Scanner");

  /* Toggle the led to indicate setup complete */
  for(int i = 0; i < 5; i++) {
    LED_ON();
    delay(700);
    LED_OFF();
    delay(300);
  }
}

/* ----------------------------------------------------- */

// the loop routine runs over and over again forever:
void loop() {
    byte error, address;
  int nDevices;
 
  Serial.println("Scanning...");
 
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
 
      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

    LED_ON();
    delay(500);
    LED_OFF();
    delay(500);

  delay(5000);           // wait 5 seconds for next scan
}

