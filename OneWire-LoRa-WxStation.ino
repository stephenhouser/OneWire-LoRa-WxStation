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
#define MAX_NUMBER_OF_DEVICES 6     // maximum number of Dallas sensors

#define DLog(x) 
//#define DLog(x) Serial.print(x)

#include <OneWire.h>
#include <OneWireDevice.h>

#include <SSD1306.h> /*https://github.com/ThingPulse/esp8266-oled-ssd1306*/
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ESPmDNS.h>

extern bool wifi_verify(int timeout);

#define DISPLAY_SDA 4
#define DISPLAY_SDC 15
#define DISPLAY_RESET 16
#define DISPLAY_ADDRESS 0x3c

SSD1306 display(DISPLAY_ADDRESS, DISPLAY_SDA, DISPLAY_SDC);
OneWire oneWire(ONEWIRE_PIN); // (a 4.7K pull-up resistor is necessary)

OneWireDevice *devices[MAX_NUMBER_OF_DEVICES];
byte numberOfDevices = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("OneWire WxStation\n");

  // Initialize OLED Display
  pinMode(DISPLAY_RESET, OUTPUT);
  digitalWrite(DISPLAY_RESET, LOW);    // set GPIO16 low to reset OLED
  delay(50); 
  digitalWrite(DISPLAY_RESET, HIGH); // while OLED is running, must set GPIO16 in high

  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);

  // Start Device Reading Task on second core
  // xTaskCreatePinnedToCore(
  //     taskUpdateDevices, /* Function to implement the task */
  //     "updateDevices ",    /* Name of the task */
  //     4000,              /* Stack size in words */
  //     NULL,              /* Task input parameter */
  //     5,                 /* Priority of the task */
  //     NULL,              /* Task handle. */
  //     1);                /* Core where the task should run */
  xTaskCreate(
      taskUpdateDevices, /* Function to implement the task */
      "updateDevices ",  /* Name of the task */
      4000,              /* Stack size in words */
      NULL,              /* Task input parameter */
      5,                 /* Priority of the task */
      NULL);              /* Task handle. */

  // xTaskCreatePinnedToCore(taskMaintainWiFi, "maintainWiFi", 1024, NULL, 2,
  //                         NULL, 1);
}

void loop() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi Searching...");
        display.drawString(0, 0, "WiFi Searching...");
        display.display();
        wifi_verify(30);
    }

    if (numberOfDevices) {
        display.clear();

        // Serial.print("\n\n");
        // Serial.print(String(millis() / 1000.0) + " sec");
        // Serial.printf(" %i OneWire devices found.\n", numberOfDevices);

        for (byte d = 0; d < numberOfDevices; d++) {
          String disp = devices[d]->toString();

          display.setTextAlignment(TEXT_ALIGN_LEFT);
          display.setFont(ArialMT_Plain_16);
          display.drawString(0, (d * 16), disp);
          display.display();

          Serial.print(disp);
          Serial.print(" ==> ");
          Serial.println(devices[d]->toJSON());
        }
      } else {
        Serial.println("No OneWire Devices.");
    }

    Serial.println("");
    //Serial.println("Core:" + String(xPortGetCoreID()));
    delay(1000);  /* every 1 second */
}

int findDevices() {
  int numberOfFoundDevices = 0;
  uint8_t address[8];
  while (oneWire.search(address) && numberOfFoundDevices < MAX_NUMBER_OF_DEVICES) {
    OneWireDevice *device = OneWireDevice::objectForDevice(&oneWire, (uint8_t *)&address);
    if (device != NULL) {
      devices[numberOfFoundDevices++] = device;
      device->begin();
    }
  }

  return numberOfFoundDevices;
}

void taskUpdateDevices(void *pvParameters) {
  Serial.println("1-Wire Searching...");
  display.drawString(0, 0, "1-Wire Searching...");
  display.display();

  numberOfDevices = findDevices();
  if (!numberOfDevices) {
    vTaskDelete(NULL);
  }

  while (1) {
    for (int d = 0; d < numberOfDevices; d++) {
      OneWireDevice *device = devices[d];
      device->update();
    }
  } 
}

void hexdump(uint8_t *data, int len) {
  for (int i = 0; i < len; i++) {
    Serial.printf("%02x ", data[i]);
    if (i % 16 == 0) {
      Serial.println("");
    }
  }
  Serial.println("");
}
