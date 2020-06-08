#include <OneWire.h>
#include <WiFi.h>
#include <U8x8lib.h>

#define DISPLAY_SDA           4
#define DISPLAY_SDC           15
#define DISPLAY_RESET         16
// #define DISPLAY_ADDRESS       0x3c

U8X8_SSD1306_128X64_NONAME_SW_I2C oled(DISPLAY_SDC, DISPLAY_SDA, DISPLAY_RESET);

//#define SHOW_DALLAS_ERROR        // uncomment to show Dallas ( CRC ) errors on Serial.
#define ONEWIRE_PIN           17   // OneWire Dallas sensors are connected to this pin
#define MAX_NUMBER_OF_SENSORS 10   // maximum number of Dallas sensors
#define OW_PARASITE_POWER     0     // 0 = off, 1 = on

struct sensorStruct {
  byte addr[8];
  String name;
  union _data {
    float temp;
    int count;
    int value;
  } data;
} sensor[MAX_NUMBER_OF_SENSORS];

OneWire  ow(ONEWIRE_PIN);        // (a 4.7K pull-up resistor is necessary)
byte numberOfSensors;


void ow_write(byte data[], int length) {
  for (byte i = 0; i < length; i++) { // we need 9 bytes
    ow.write(data[i]);
  }
}

void ow_read(byte data[], int length) {
  memset(data, 0, length);
  for (byte i = 0; i < length; i++) { // we need 9 bytes
    data[i] = ow.read();
    // Serial.print(data[i], HEX);
    // Serial.print(" ");
    // if (((i + 1) % 16) == 0) {
    //   Serial.println("");
    // }
  }
  // Serial.println("");
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
  while (ow.search(currentAddr) && numberOfFoundSensors < MAX_NUMBER_OF_SENSORS) {
    for (byte i = 0; i < 8; i++) {
      sensor[numberOfFoundSensors].addr[i] = currentAddr[i];
    }

    String chip = "";
    switch (currentAddr[0]) {
        case 0x10:
          chip = "DS18S20 Temperature";  // or old DS1820
          break;
        case 0x1D:
          chip = "DS2423 RAM/Counter";
          break;
        case 0x20:
          chip = "DS2450 Quad A/D";
          break;
        case 0x22:
          chip = "DS1822 Temperature";
          break;
        case 0x28:
          chip = "DS18B20 Temperature";
          break;
        case 0x29:
          chip = "DS2408 8 Switch";
          break;
        default:
          //continue;
          chip = "Unknown";
          break;
    }
        
    sensor[numberOfFoundSensors].name = chip + "." + String(numberOfFoundSensors, DEC);
    numberOfFoundSensors++;
  }
  
  return numberOfFoundSensors;  
}

int16_t read_DSTemperature(byte address[]) {
  byte device_type;
  switch (address[0]) {
    case 0x10: // DS18S20
      device_type = 1;
      break;
    case 0x22: // DS1822
    case 0x28: // DS18B20
      device_type = 0;
      break;
    default:
      return 0;
  }

  bool present = ow.reset();
  if (present) {
    byte data[12];

    ow.select(address);
    ow.write(0x44, OW_PARASITE_POWER);        // start conversion, with parasite power off at the end
    vTaskDelay(750 / portTICK_PERIOD_MS);     //wait for conversion ready

    present = ow.reset();
    if (present) {
      ow.select(address);
      ow.write(0xBE);         // Read Scratchpad
      ow_read(data, 9);
      if (OneWire::crc8(data, 8) == data[8]) {
        ow.reset();

        int16_t raw = (data[1] << 8) | data[0];
        if (device_type) {
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

        return raw;        
      }
    }
  }

  // CRC of temperature reading indicates an error, so we print a error message and discard this reading
  Serial.print("ERROR: read_DSTemperature() Not present or CRC not valid");
  return 0;
}

bool crc8(byte data[], int length) {
    return OneWire::crc8(data, (length - 1)) == data[(length - 1)];
}

bool crc16(byte data[], int length) {
    uint16_t crc = OneWire::crc16(data, (length - 2));
    uint8_t *crcBytes = (uint8_t *)&crc;
    uint8_t crcLo = ~data[(length - 2)];
    uint8_t crcHi = ~data[(length - 1)];
    return !((crcLo != crcBytes[0]) || (crcHi != crcBytes[1]));
}

// https://github.com/jbechter/arduino-onewire-DS2423
#define DS2423_READ_MEMORY_COMMAND 0xa5
#define DS2423_PAGE_ONE 0xc0
#define DS2423_PAGE_TWO 0xe0
#define DS2423_COUNTER_A 0x01
#define DS2423_COUNTER_B 0x02

int16_t read_DS2423(byte address[]) {
  if (address[0] == 0x1D) {
    uint8_t data[45];

    data[0] = DS2423_READ_MEMORY_COMMAND;
    data[1] = DS2423_PAGE_ONE;
    data[2] = 0x01;
    ow.reset();
    ow.select(address);
    ow_write(data, 3);
    ow_read(data + 3, 42);
    if (crc16(data, 45)) {
      ow.reset();
      uint32_t count = (uint32_t)data[38];
      for (int j = 37; j >= 35; j--) {
          count = (count << 8) + (uint32_t)data[j];
      }

      return count;
    }
  }

  Serial.print("ERROR: read_DS2423() Not present or CRC not valid");
  return 0;
}

int16_t read_DS2408(byte address[]) {
  if (address[0] == 0x29) {
    bool present = ow.reset();
    if (present) {
      byte data[13];

      ow.select(address);

      data[0] = 0xF0;    // Read PIO Registers
      data[1] = 0x88;    // LSB address
      data[2] = 0x00;    // MSB address
      ow_write(data, 3);
      ow_read(data + 3, 10);
      if (crc8(data, 13)) {     // 3 cmd bytes, 6 data bytes, 2 0xFF, 2 CRC16
        ow.reset();
        Serial.print("  data =");
        for(uint8_t index = 0; index < 11; index++) {
            Serial.print(" ");
            Serial.print(data[index], HEX);
        }

        return 0;        
      }
    }
  }

  Serial.print("ERROR: read_DS2408() Not present or CRC not valid");
  return 0;
}

void taskReadSensors(void * pvParameters) {
  numberOfSensors = findOneWireDevices();
  if (!numberOfSensors) {
    vTaskDelete(NULL);
  }

  while (TRUE) {
    for (byte thisSensor = 0; thisSensor < numberOfSensors; thisSensor++) {
      // Serial.println(sensor[thisSensor].name);

      switch (sensor[thisSensor].addr[0]) { // the first ROM byte indicates which chip
        case 0x10:  // DS18S20 or old DS1820
        case 0x22:  // DS1822
        case 0x28:  // DS18B20
          sensor[thisSensor].data.temp = read_DSTemperature(sensor[thisSensor].addr);
          break;
        case 0x29:  // DS2408
          sensor[thisSensor].data.value = read_DS2408(sensor[thisSensor].addr);
          break;
        case 0x1D:
          sensor[thisSensor].data.count = read_DS2423(sensor[thisSensor].addr);
          break;
        default:
          continue;
      }    
    }
  }
}

unsigned long last_ws_time = 0;
uint32_t last_ws_count = 0;
double rps_a[5];
uint8_t  rps_i = 0;

float wind_speed(uint32_t count) {
  uint32_t elapsed_ms = millis() - last_ws_time;
  uint32_t elapsed_count = count - last_ws_count;

  // if (elapsed_ms > 3000) {  // 5 seconds
    last_ws_time = millis();
    last_ws_count = count;
  // }


  rps_a[rps_i] = ((elapsed_count * 1000.0) / elapsed_ms) / 2.0;;
  rps_i = (rps_i + 1) % 5;

  double rps = 0;
  for (int i = 0; i < 5; i++) {
    rps += rps_a[i];
  }
  rps /= 5;

  if (rps >= 0.0 && ((rps * 2.453) < 200)) {
     return (rps * 2.453) * 0.447040972; /* convert mph to m/s*/
  }

  return 0.0;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting...");

  oled.begin();
  oled.setFont(u8x8_font_8x13_1x2_f);
  oled.setCursor(0, 0);
  oled.print("Starting...\n!");

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
}

void loop() {
  if (numberOfSensors) {
    // display.clear();

    Serial.print("\n\n");
    Serial.print(String(millis() / 1000.0) + " sec");
    Serial.printf(" %i Dallas sensors found.\n", numberOfSensors);
    
    for (byte thisSensor = 0; thisSensor < numberOfSensors; thisSensor++) {
      printAddress(sensor[thisSensor].addr);

      switch (sensor[thisSensor].addr[0]) {
          case 0x10:
          case 0x22:
          case 0x28:
            Serial.println(sensor[thisSensor].name + ": " + String((float)sensor[thisSensor].data.temp / 16.0) + "°C");
            break;
          case 0x1D:  // "DS2423 RAM/Counter";
            Serial.println(sensor[thisSensor].name + ": " + String(sensor[thisSensor].data.count));
            {
              float ws = wind_speed(sensor[thisSensor].data.count);
              Serial.println("\t: " + String(ws) + " m/s");
            }
            break;
          case 0x20:  // "DS2450 Quad A/D";
          case 0x29:  // "DS2408 8 Switch";
            Serial.println(sensor[thisSensor].name + ": " + String(sensor[thisSensor].data.value));
            break;
          default:
            //continue;
            Serial.println(sensor[thisSensor].name + ": " + String(sensor[thisSensor].data.value));
            break;
      }
    }
  } else {
    Serial.println("No Dallas sensors.");
  }
  
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}


