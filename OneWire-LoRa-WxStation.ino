/*************************************************************************************************
 * 
 *     Read multiple DS18B20 Dallas sensors in a task.
 *     
 *     For ESP32 by CelliesProjects 2017.
 *     
 *     Code adapted from https://www.pjrc.com/teensy/td_libs_OneWire.html
 * 
 ************************************************************************************************/
//#define SHOW_DALLAS_ERROR        // uncomment to show Dallas ( CRC ) errors on Serial.
#define ONEWIRE_PIN           17    // OneWire Dallas sensors are connected to this pin
#define MAX_NUMBER_OF_SENSORS 6    // maximum number of Dallas sensors

#include <OneWire.h>
#include <WiFi.h>
#include <SSD1306.h>

#define DISPLAY_SDA 4
#define DISPLAY_SDC 15
#define DISPLAY_RESET 16
#define DISPLAY_ADDRESS 0x3c

SSD1306  display(DISPLAY_ADDRESS, DISPLAY_SDA, DISPLAY_SDC);

const char* ssid = "houser";
const char* password = ""; //"----------";

OneWire  ds(ONEWIRE_PIN);        // (a 4.7K pull-up resistor is necessary)

struct sensorStruct {
  byte addr[8];
  float temp;
  String name;
} sensor[MAX_NUMBER_OF_SENSORS];

byte numberOfSensors;

void setup() {
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }
  Serial.begin(115200);
  Serial.println("\n\nMultiple DS18B20 sensors as task ESP32 example.");
  Serial.printf("Connecting to %s with password %s\n", ssid,  password);

  pinMode(DISPLAY_RESET, OUTPUT);
  digitalWrite(DISPLAY_RESET, LOW);    // set GPIO16 low to reset OLED
  delay(50); 
  digitalWrite(DISPLAY_RESET, HIGH); // while OLED is running, must set GPIO16 in high
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);

  xTaskCreatePinnedToCore(
    taskReadSensors,                /* Function to implement the task */
    "readSensors ",                 /* Name of the task */
    4000,                           /* Stack size in words */
    NULL,                           /* Task input parameter */
    5,                              /* Priority of the task */
    NULL,                           /* Task handle. */
    1);                             /* Core where the task should run */

  // WiFi.mode(WIFI_STA);
  // WiFi.begin(ssid, password);
  // while ( WiFi.status() != WL_CONNECTED ) {
  //   vTaskDelay( 250 /portTICK_PERIOD_MS );
  //   Serial.print( "." );
  // }
  display.drawString(0, 0, "Searching...");
  display.display();

  Serial.println();
}

void loop() {
  if (numberOfSensors) {
    display.clear();

    Serial.print("\n\n");
    Serial.print(String(millis() / 1000.0) + " sec");
    Serial.printf(" %i Dallas sensors found.\n", numberOfSensors);
    
    for (byte thisSensor = 0; thisSensor < numberOfSensors; thisSensor++) {
      float celsius = (float)sensor[thisSensor].temp / 16.0;
      float fahrenheit = celsius * 1.8 + 32.0;
      String disp = String(fahrenheit, 2) + "°F " + String(celsius, 2) + "°C";
      display.setTextAlignment(TEXT_ALIGN_LEFT);
      display.setFont(ArialMT_Plain_16);
      display.drawString(0, thisSensor*16, disp);
      display.display();

      printAddress(sensor[thisSensor].addr);
      Serial.println(sensor[thisSensor].name + ": " + String((float)sensor[thisSensor].temp / 16.0) + "°C");
    }
  } else {
    Serial.println("No Dallas sensors.");
  }
  
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void printAddress(byte address[8]) {
    for (uint8_t i = 0; i < 8; i++) {
    if (address[i] < 0x10) {
      Serial.print("0");
    }
    
    Serial.print(address[i], HEX);
    Serial.print(" ");
  }  
}

int findOneWireDevices() {
  int numberOfFoundSensors = 0;
  byte currentAddr[8];
  while (ds.search(currentAddr) && numberOfFoundSensors < MAX_NUMBER_OF_SENSORS) {
    for (byte i = 0; i < 8; i++) {
      sensor[numberOfFoundSensors].addr[i] = currentAddr[i];
    }

    String chip = "";
    switch (currentAddr[0]) {
        case 0x10:
          chip = "DS18S20 Temp";  // or old DS1820
          break;
        case 0x1D:
          chip = "DS2423 RAM/Counter";
          break;
        case 0x20:
          chip = "DS2450 Quad A/D";
          break;
        case 0x22:
          chip = "DS1822 Temp";
          break;
        case 0x28:
          chip = "DS18B20 Temp";
          break;
        case 0x29:
          chip = "DS2408 8 Switch";
          break;
        default:
          //continue;
          break;
    }
        
    sensor[numberOfFoundSensors].name = chip + "." + String(numberOfFoundSensors, DEC);
    numberOfFoundSensors++;
  }
  
  return numberOfFoundSensors;  
}

void taskReadSensors(void * pvParameters) {
  numberOfSensors = findOneWireDevices();
  if (!numberOfSensors) {
    vTaskDelete( NULL );
  }

  while (1) {
    for (byte thisSensor = 0; thisSensor < numberOfSensors; thisSensor++) {
      byte type_s;
      // the first ROM byte indicates which chip
      switch (sensor[thisSensor].addr[0]) {
        case 0x10:
          //Serial.println("  Chip = DS18S20");  // or old DS1820
          type_s = 1;
          break;
        case 0x28:
          //Serial.println("  Chip = DS18B20");
          type_s = 0;
          break;
        case 0x22:
          //Serial.println("  Chip = DS1822");
          type_s = 0;
          break;
        default:
          continue;
      }

      ds.reset();
      ds.select(sensor[thisSensor].addr);
      ds.write(0x44, 0);        // start conversion, with parasite power off at the end

      vTaskDelay(750 / portTICK_PERIOD_MS); //wait for conversion ready

      byte data[12];
      byte present = ds.reset();
      ds.select(sensor[thisSensor].addr);
      ds.write(0xBE);         // Read Scratchpad

//      Serial.print( sensor[thisSensor].name ); 
//      Serial.print("  Data = ");
//      Serial.print( present, HEX );
//      Serial.print(": ");
      for ( byte i = 0; i < 9; i++) { // we need 9 bytes
        data[i] = 0;
        data[i] = ds.read();
//        Serial.print(data[i], HEX);
//        Serial.print(" ");
      }
      
//      Serial.println();

      if (OneWire::crc8(data, 8) != data[8]) {
        // CRC of temperature reading indicates an error, so we print a error message and discard this reading
        Serial.print( millis() / 1000.0 ); Serial.print( " - CRC error from device " ); Serial.println( thisSensor );
      } else {
        int16_t raw = (data[1] << 8) | data[0];
        if (type_s) {
          raw = raw << 3; // 9 bit resolution default
          if (data[7] == 0x10) {
            // "count remain" gives full 12 bit resolution
            raw = (raw & 0xFFF0) + 12 - data[6];
          }
        } else {
          byte cfg = (data[4] & 0x60);
          // at lower res, the low bits are undefined, so let's zero them
          if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
          else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
          else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
          //// default is 12 bit resolution, 750 ms conversion time
        }
        
        //Serial.println(raw);
        sensor[thisSensor].temp = raw;
      }
    }
  }
}
