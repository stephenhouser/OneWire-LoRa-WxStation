/*
 * OneWire Weather Station for ESP32
 */

#include <WiFi.h>

// === OLED Display on ESP32
#define OLED_ENABLED

#if defined(OLED_ENABLED)
#include <U8x8lib.h>

#define DISPLAY_SDA           4
#define DISPLAY_SDC           15
#define DISPLAY_RESET         16

U8X8_SSD1306_128X64_NONAME_SW_I2C oled_display(DISPLAY_SDC, DISPLAY_SDA, DISPLAY_RESET);
#endif

// One Wire Bus; connections, object, and utility routines
#include <OneWire.h>

#define ONEWIRE_PIN           17   // OneWire Dallas sensors are connected to this pin
#define MAX_NUMBER_OF_SENSORS 10   // maximum number of Dallas sensors
#define OW_PARASITE_POWER     1     // 0 = off, 1 = on

OneWire  ow(ONEWIRE_PIN);        // (a 4.7K pull-up resistor is necessary)

void ow_write(byte data[], int length) {
  for (byte i = 0; i < length; i++) {
    ow.write(data[i], OW_PARASITE_POWER);
  }
}

void ow_read(byte data[], int length) {
  memset(data, 0, length);
  for (byte i = 0; i < length; i++) {
    data[i] = ow.read();
  }
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

void ow_print_address(byte address[8]) {
    for (uint8_t i = 0; i < 8; i++) {
    if (address[i] < 0x10) {
      Serial.print("0");
    }
    
    Serial.print(address[i], HEX);
    Serial.print(" ");
  }  
}

struct sensorStruct {
  byte addr[8];
  String name;
  union _data {
    float temperature;
    int value;
    int count;
    int voltage[4];
  } data;
} sensor[MAX_NUMBER_OF_SENSORS];

byte numberOfSensors;

// == Dallas Semiconductior OneWire Devices...

/*
 * DS18S20, DS18B20 Temperature -- used for Temperature
 */
float read_DS18x20(byte address[]) {
  bool present = ow.reset();
  if (present) {
    byte data[12];

    ow.select(address);
    ow.write(0x44, OW_PARASITE_POWER);        // start conversion, with parasite power off at the end

    vTaskDelay(1 * portTICK_PERIOD_MS);     //wait for conversion ready

    present = ow.reset();
    if (present) {
      ow.select(address);
      ow.write(0xBE, OW_PARASITE_POWER);         // Read Scratchpad

      ow_read(data, 9);
      if (OneWire::crc8(data, 8) == data[8]) {

        // Convert the data to actual temperature
        // because the result is a 16 bit signed integer, it should
        // be stored to an "int16_t" type, which is always 16 bits
        // even when compiled on a 32 bit processor.
        int16_t raw = (data[1] << 8) | data[0];
        if (address[0] == 0x10) {
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
        float celsius = (float)raw / 16.0;
        // fahrenheit = celsius * 1.8 + 32.0;

        return celsius;
      }
    }
  }

  // CRC of temperature reading indicates an error, so we print a error message and discard this reading
  Serial.print("ERROR: read_DSTemperature() Not present or CRC not valid");
  return 0.0;
}

/*
 * DS2423 Counter -- Used for Wind Speed
 * 
 * https://github.com/jbechter/arduino-onewire-DS2423
 */
int16_t read_DS2423(byte address[]) {
  if (address[0] == 0x1D) {
    bool present = ow.reset();
    if (present) {
      uint8_t data[45];
      data[0] = 0xa5; // DS2423_READ_MEMORY_COMMAND
      data[1] = 0xc0; // DS2423_PAGE_ONE;
      data[2] = 0x01;

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
  }

  Serial.print("ERROR: read_DS2423() Not present or CRC not valid");
  return 0;
}

/*
 * DS2408  -- Used for Wind Direction on v1.
 */
int16_t read_DS2408(byte address[]) {
  if (address[0] == 0x29) {
    bool present = ow.reset();
    if (present) {
      byte data[13];
      data[0] = 0xF0;    // Read PIO Registers
      data[1] = 0x88;    // LSB address
      data[2] = 0x00;    // MSB address

      ow.select(address);
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

/*
 * DS2450 Quad A/D Converter -- Used for Wind Direction on v2 and v3.
 */
#define DS2450_WRITE_COMMAND 0x55
#define DS2450_BEGIN_VOLTAGE_CONVERSION 0x3c
#define DS2450_REGISTER_4CHANNEL 0x0f
#define DS2450_READOUT_CONTROL 0xaa
#define DS2450_MEMORY_READ_COMMAND 0xaa
#define DS2450_8_BIT_RESOLUTION 0x08
#define DS2450_POR_OFF_NO_ALARMS_5V_RANGE 0x01
#define DS2450_VCC_CONTROL_BYTE_ADDR_LO 0x1c
#define DS2450_VCC_CONTROL_BYTE_ADDR_HI 0x00
#define DS2450_VCC_POWERED 0x40
#define DS2450_CONTROL_STATUS_DATA_ADDR_LO 0x08
#define DS2450_CONTROL_STATUS_DATA_ADDR_HI 0x00
#define DS2450_CONVERSION_COMPLETE 0xff
#define DS2450_AD_CHANNELS_ADDR_LO 0x00
#define DS2450_AD_CHANNELS_ADDR_HI 0x00
#define DS2450_TEMP_CONVERSION_TIMEOUT 50

void setup_DS2450(byte address[]) {
   ow.reset();
   ow.select(address);
   ow.write(DS2450_WRITE_COMMAND, OW_PARASITE_POWER);
   ow.write(DS2450_VCC_CONTROL_BYTE_ADDR_LO, OW_PARASITE_POWER);
   ow.write(DS2450_VCC_CONTROL_BYTE_ADDR_HI, OW_PARASITE_POWER);
   ow.write(0x00, OW_PARASITE_POWER);
   ow.read(); // crc lsb
   ow.read(); // crc msb
   ow.read(); // verify

   ow.reset();
   ow.select(address);
   ow.write(DS2450_WRITE_COMMAND, OW_PARASITE_POWER);
   ow.write(DS2450_CONTROL_STATUS_DATA_ADDR_LO, OW_PARASITE_POWER);
   ow.write(DS2450_CONTROL_STATUS_DATA_ADDR_HI, OW_PARASITE_POWER);
    for (int i = 0; i < 4; i++) {
      ow.write(0 /*DS2450_8_BIT_RESOLUTION*/, OW_PARASITE_POWER);
      ow.read(); // crc
      ow.read();
      ow.read(); // verify data
      ow.write(DS2450_POR_OFF_NO_ALARMS_5V_RANGE, OW_PARASITE_POWER);
      ow.read(); // crc
      ow.read();
      ow.read(); // verify data
    }

  Serial.println("SETUP DS2450 Complete.");
}

int16_t read_DS2450(byte address[], int voltage[]) {
  if (address[0] == 0x20) {
    // Serial.print("read_DS2450: ");
    bool present = ow.reset();
    if (present) {
      byte data[13];
      data[0] = DS2450_BEGIN_VOLTAGE_CONVERSION;    // Read PIO Registers
      data[1] = DS2450_REGISTER_4CHANNEL;    // LSB address
      data[2] = DS2450_READOUT_CONTROL;    // MSB address

      ow.select(address);
      ow_write(data, 3);
      ow_read(data + 3, 2); // CRC16 of command byte, select and control.

      vTaskDelay(1 * portTICK_PERIOD_MS);     //wait for conversion ready

      if (ow.reset()) {
        ow.select(address);
        data[0] = DS2450_MEMORY_READ_COMMAND;    // Read PIO Registers
        data[1] = DS2450_AD_CHANNELS_ADDR_LO;    // LSB address
        data[2] = DS2450_AD_CHANNELS_ADDR_HI;    // MSB address
        ow_write(data, 3);
        ow_read(data + 3, 10);
        if (crc16(data, 13)) {
            // float voltage[4];
            // voltage[0] = (float)((data[4] << 8) | data[3]) / 12580.0;
            // voltage[1] = (float)((data[6] << 8) | data[5]) / 12580.0;
            // voltage[2] = (float)((data[8] << 8) | data[7]) / 12580.0;
            // voltage[3] = (float)((data[10] << 8) | data[9]) / 12580.0;
            voltage[0] = ((data[4] << 8) | data[3]);
            voltage[1] = ((data[6] << 8) | data[5]);
            voltage[2] = ((data[8] << 8) | data[7]);
            voltage[3] = ((data[10] << 8) | data[9]);
            return (int)voltage[0];
        }
      }
    }
  }

  Serial.print("ERROR: read_DS2450() Not present or CRC not valid");
  return 0;
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
          setup_DS2450(currentAddr);
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


unsigned long last_ws_time = 0;
uint32_t last_ws_count = 0;
float last_ws_reading = 0;
float wind_speed(uint32_t count) {
  if (last_ws_time == 0) {
    last_ws_time = millis();
  }

  uint32_t elapsed_ms = millis() - last_ws_time;
  if (elapsed_ms > 10000) {  // 10 seconds
    uint32_t elapsed_count = count - last_ws_count;
    last_ws_time = millis();
    last_ws_count = count;

    // from AAGElectroinca Source for 1Wire Weather Station, TI8515.java
    last_ws_reading = (float)((((elapsed_count * 1000.0)/(elapsed_ms)) /2.0) * 2.453) * 1862 / 3600; 
    if (last_ws_reading > 200) {
      last_ws_reading = 0;
    }
  }

  return last_ws_reading;
}

/*
 * Decode Wind Direction from quad voltages reported by DS2450
 * http://sheepdogsoftware.co.uk/sc3wmw.htm
 */
int wind_direction_decode(int voltage) {
   if (voltage > 47000) return 2;
   if (voltage > 15000) return 1;
   return 0;
}

String wind_direction_rose(int voltage[4]) {
  // Wind direction can be considered a base 3 number with four digits
  int direction = wind_direction_decode(voltage[0]) * 27
    + wind_direction_decode(voltage[1]) * 9
    + wind_direction_decode(voltage[2]) * 3
    + wind_direction_decode(voltage[3]);

  switch (direction) {
    case 8:  return "ENE";  // 292.5;
    case 25: return "ESE";  // 247.5;
    case 26: return "E";    // 270.0;
    case 44: return "WSW";  // 112.5;
    case 51: return "WNW";  //  67.5;
    case 53: return "W";    //  90.0;
    case 56: return "NNE";  // 337.5;
    case 62: return "NE";   // 315.0;
    case 68: return "SSW";  // 157.5;
    case 71: return "SW";   // 135.0;
    case 72: return "NNW";  //  22.5;
    case 74: return "N";    //   0.0;
    case 76: return "SSE";  // 202.5;
    case 77: return "S";    // 180.0;
    case 78: return "NW";  //  45.0;
    case 79: return "SE";  // 225.0;
  }

  Serial.println("DIR=" + String(direction));
  return "Err";
}

float wind_direction(int voltage[4]) {
  // Wind direction can be considered a base 3 number with four digits
  int direction = wind_direction_decode(voltage[0]) * 27
    + wind_direction_decode(voltage[1]) * 9
    + wind_direction_decode(voltage[2]) * 3
    + wind_direction_decode(voltage[3]);

  switch (direction) {
    case 8:  return 292.5; // ENE
    case 25: return 247.5; // ESE
    case 26: return 270.0; // E
    case 44: return 112.5; // WSW
    case 51: return  67.5; // WNW
    case 53: return  90.0; // W
    case 56: return 337.5; // NNE
    case 62: return 315.0; // NE
    case 68: return 157.5; // SSW
    case 71: return 135.0; // SW
    case 72: return  22.5; // NNW
    case 74: return   0.0; // N
    case 76: return 202.5; // SSE
    case 77: return 180.0; // S
    case 78: return  45.0; // NW
    case 79: return 225.0; // SE
  }

  Serial.println("DIR=" + String(direction));
  return -1.0;
}

void taskReadSensors(void * pvParameters) {
  numberOfSensors = findOneWireDevices();
  if (!numberOfSensors) {
    vTaskDelete(NULL);
  }

  while (1 == 1) {
    for (byte thisSensor = 0; thisSensor < numberOfSensors; thisSensor++) {
      switch (sensor[thisSensor].addr[0]) { // the first ROM byte indicates which chip
        case 0x10:  // DS18S20 or old DS1820
        case 0x22:  // DS1822
        case 0x28:  // DS18B20
          sensor[thisSensor].data.temperature = read_DS18x20(sensor[thisSensor].addr);
          break;
        case 0x29:  // DS2408
          sensor[thisSensor].data.value = read_DS2408(sensor[thisSensor].addr);
          break;
        case 0x1D:
          sensor[thisSensor].data.count = read_DS2423(sensor[thisSensor].addr);
          break;
        case 0x20:  // DS2450
          read_DS2450(sensor[thisSensor].addr, sensor[thisSensor].data.voltage);
          break;
        default:
          continue;
      }    
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS); // 15 seconds
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting...");

  #if defined(OLED_ENABLED)
  oled_display.begin();
  oled_display.setFont(u8x8_font_8x13_1x2_f);
  oled_display.setCursor(0, 0);
  oled_display.print("Starting...\n");
  #endif

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

//
//
//
//

void loop() {
  if (numberOfSensors) {
    #if defined(OLED_ENABLED)
    oled_display.clear();
    #endif

    Serial.print("\n\n");
    Serial.print(String(millis() / 1000.0) + " sec");
    Serial.printf(" %i Dallas sensors found.\n", numberOfSensors);
    
    for (byte thisSensor = 0; thisSensor < numberOfSensors; thisSensor++) {
      ow_print_address(sensor[thisSensor].addr);

      switch (sensor[thisSensor].addr[0]) {
          case 0x10:
          case 0x22:
          case 0x28:
            Serial.println(sensor[thisSensor].name + ": " + String(sensor[thisSensor].data.temperature) + "°C");
            break;
          case 0x1D:  // "DS2423 RAM/Counter";
            Serial.println(sensor[thisSensor].name + ": " + String(sensor[thisSensor].data.count));
            {
              float ws = wind_speed(sensor[thisSensor].data.count);
              String ws_line = String(ws * 0.62137120) + " mph";
              Serial.println(ws_line);
              #if defined(OLED_ENABLED)
              oled_display.drawString(5, 6, ws_line.c_str());
              #endif
            }
            break;
          case 0x20:  // "DS2450 Quad A/D";     
            Serial.println(sensor[thisSensor].name + ": " + String(sensor[thisSensor].data.value));
            {
              String wd = wind_direction_rose(sensor[thisSensor].data.voltage);
              Serial.println("\t: " + wd);
              #if defined(OLED_ENABLED)
              oled_display.drawString(0, 6, wd.c_str());
              #endif
            }
            break;

          case 0x29:  // "DS2408 8 Switch";
            Serial.println(sensor[thisSensor].name + ": " + String(sensor[thisSensor].data.value));
            break;

          default:
            Serial.println(sensor[thisSensor].name + ": " + String(sensor[thisSensor].data.value));
            break;
      }
    }
  } else {
    Serial.println("No Dallas sensors.");
  }
  
  vTaskDelay(15000 / portTICK_PERIOD_MS); // 15 seconds
}


