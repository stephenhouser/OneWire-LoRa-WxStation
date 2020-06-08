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

void printAddress(byte address[8]) {
    for (uint8_t i = 0; i < 8; i++) {
    if (address[i] < 0x10) {
      Serial.print("0");
    }
    
    Serial.print(address[i], HEX);
    Serial.print(" ");
  }  
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
   ow.write(DS2450_WRITE_COMMAND, 0);
   ow.write(DS2450_VCC_CONTROL_BYTE_ADDR_LO, 0);
   ow.write(DS2450_VCC_CONTROL_BYTE_ADDR_HI, 0);
   ow.write(0x00, 0);
   ow.read(); // crc lsb
   ow.read(); // crc msb
   ow.read(); // verify

   ow.reset();
   ow.select(address);
   ow.write(DS2450_WRITE_COMMAND, 0);
   ow.write(DS2450_CONTROL_STATUS_DATA_ADDR_LO, 0);
   ow.write(DS2450_CONTROL_STATUS_DATA_ADDR_HI, 0);
    for (int i = 0; i < 4; i++) {
      ow.write(0/*DS2450_8_BIT_RESOLUTION*/, 0);
      ow.read(); // crc
      ow.read();
      ow.read(); // verify data
      ow.write(DS2450_POR_OFF_NO_ALARMS_5V_RANGE, 0);
      ow.read(); // crc
      ow.read();
      ow.read(); // verify data
    }
  Serial.println("SETUP DS2450 Complete.");
}

#if 0
String wind_dir(float voltages[]) {
  float Va = voltages[0];
  float Vb = voltages[1];
  float Vc = voltages[2];
  float Vd = voltages[3];

  float Vmax=Va;
  if (Vb>Vmax) Vmax=Vb;
  if (Vc>Vmax) Vmax=Vc;
  if (Vd>Vmax) Vmax=Vd;
  
  float VL1=Vmax*0.83;
  float VL2=Vmax*0.58;
  float VL3=Vmax*0.25;


    a=5;
    if (Va<VL1) a=3;
    if (Va<VL2) a=2;
    if (Va<VL3) a=0;

    b=5;
    if (Vb<VL1) b=3;
    if (Vb<VL2) b=2;
    if (Vb<VL3) b=0;

    c=5;
    if (Vc<VL1) c=3;
    if (Vc<VL2) c=2;
    if (Vc<VL3) c=0;

    d=5;
    if (Vd<VL1) d=3;
    if (Vd<VL2) d=2;
    if (Vd<VL3) d=0;


      // N voltage=2.78, 3.95, 4.97, 4.35
      //   voltage=2.61, 4.28, 4.59, 3.05
      // E voltage=2.93, 4.84, 3.94, 3.48
      //   voltage=4.56, 4.60, 4.05, 3.47
      // S voltage=4.56, 4.88, 0.16, 3.48
      // W voltage=0.11, 3.96, 2.74, 4.33


    private String[] tabla={"5255", "5335","5525","5533", 
                            "5552", "0552","0555","0055", 
                            "5055", "5005","5505","5500",
                            "5550", "2550","2555","3355"};
                            
    private String[] roseta = {"ERR"," N ", "NNW", "NW ", "WNW",
                                     " W ", "WSW", "SW ", "SSW",
                                     " S ", "SSE", "SE" , "ESE",
                                     " E ", "ENE", "NE ", "NNE" };                            
    

    pos=String.valueOf(a)+String.valueOf(b)+String.valueOf(c)+String.valueOf(d);
    for (int i=0;i<=15;i++){
      if (pos.equals(tabla[i]))  result=i+1; 
    }  
}
#endif 

int16_t read_DS2450(byte address[]) {
  if (address[0] == 0x20) {
    // Serial.print("read_DS2450: ");
    bool present = ow.reset();
    if (present) {
      byte data[13];

      ow.select(address);
      data[0] = DS2450_BEGIN_VOLTAGE_CONVERSION;    // Read PIO Registers
      data[1] = DS2450_REGISTER_4CHANNEL;    // LSB address
      data[2] = DS2450_READOUT_CONTROL;    // MSB address
      ow_write(data, 3);

      vTaskDelay(10 * portTICK_PERIOD_MS);     //wait for conversion ready

      ow_read(data + 3, 2);

      // char line[256];
      // snprintf(line, 256, "PRE: %2.2x %2.2x", data[3], data[4]);
      // Serial.println(line);

      if (ow.reset()) {
        ow.select(address);
        data[0] = DS2450_MEMORY_READ_COMMAND;    // Read PIO Registers
        data[1] = DS2450_AD_CHANNELS_ADDR_LO;    // LSB address
        data[2] = DS2450_AD_CHANNELS_ADDR_HI;    // MSB address
        ow_write(data, 3);
        ow_read(data + 3, 10);

        // Serial.print("READ: ");
        // for (int i = 0; i < 13; i++) {
        //     Serial.print(String(data[i], HEX) + " ");
        // }
        // Serial.println("");

        if (crc16(data, 13)) {
            float voltage[4];
            voltage[0] = (float)((data[4] << 8) | data[3]) / 12580.0;
            voltage[1] = (float)((data[6] << 8) | data[5]) / 12580.0;
            voltage[2] = (float)((data[8] << 8) | data[7]) / 12580.0;
            voltage[3] = (float)((data[10] << 8) | data[9]) / 12580.0;
            Serial.print("voltage=");
            Serial.print(voltage[0]);
            Serial.print(", ");
            Serial.print(voltage[1]);
            Serial.print(", ");
            Serial.print(voltage[2]);
            Serial.print(", ");
            Serial.print(voltage[3]);
            Serial.println("");

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
        case 0x20:  // DS2450
          sensor[thisSensor].data.value = read_DS2450(sensor[thisSensor].addr);
          break;
        default:
          continue;
      }    
    }

    //vTaskDelay(15000 / portTICK_PERIOD_MS); // 15 seconds
  }
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
              Serial.println("\t: " + String(ws * 0.62137120) + " mph");
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
  
  vTaskDelay(15000 / portTICK_PERIOD_MS); // 15 seconds
}


