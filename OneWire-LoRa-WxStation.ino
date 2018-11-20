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

#include <OneWire.h>
#include <OneWireDevice.h>

#include <SSD1306.h> /*https://github.com/ThingPulse/esp8266-oled-ssd1306*/
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ESPmDNS.h>

extern bool wifi_verify(int timeout);

#define SENSOR_SAMPLE_SECONDS 1
#define WIFI_CHECK_TIMEOUT_SECONDS  30

#define DISPLAY_SDA 4
#define DISPLAY_SDC 15
#define DISPLAY_RESET 16
#define DISPLAY_ADDRESS 0x3c

#define MQTT_SERVER "192.168.69.4"
#define MQTT_PORT 1883
#define MQTT_CLIENTID "WxStation"
#define MQTT_USERNAME "writer"

#define MQTT_TOPIC MQTT_CLIENTID
#define MQTT_PUBLISH_INTERVAL_SECONDS  60

SSD1306 display(DISPLAY_ADDRESS, DISPLAY_SDA, DISPLAY_SDC);
OneWire oneWire(ONEWIRE_PIN); // (a 4.7K pull-up resistor is necessary)

OneWireDevice *devices[MAX_NUMBER_OF_DEVICES];
byte numberOfDevices = 0;

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

void setup() {
    Serial.begin(115200);
    Serial.println("OneWire WxStation\n");

    setup_display();
    setup_sensors();
    setup_wifi();
    setup_mqtt();
}

void loop() {
    loop_display();
    //loop_sensors(); <-- this is done in a seprate task on ESP32
    loop_wifi();
    loop_mqtt();

    delay(1000); /* every 1 second */
}

// ==== DISPLAY ====

bool setup_display() {
    // Initialize OLED Display
    pinMode(DISPLAY_RESET, OUTPUT);
    digitalWrite(DISPLAY_RESET, LOW); // set GPIO16 low to reset OLED
    delay(50);
    digitalWrite(DISPLAY_RESET, HIGH); // while OLED is running, must set GPIO16 in high

    display.init();
    display.flipScreenVertically();
    display.setFont(ArialMT_Plain_10);

    return true;
}

void loop_display() {
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_16);

    // Serial.println("Timestamp:" + String(millis()));

    if (WiFi.status() != WL_CONNECTED) {
        display.drawCircle(128 - 3, 3, 2);
    } else {
        display.fillCircle(128 - 3, 3, 3);
    }

    if (numberOfDevices) {
        for (byte d = 0; d < numberOfDevices; d++) {
            String disp = devices[d]->toString();

            display.drawString(0, (d * 16), disp);

            // Serial.print(disp);
            // Serial.print(" ==> ");
            // Serial.println(devices[d]->toJSON());
        }
    } else {
        display.drawString(0, 0, "No Devices");
        Serial.println("No Devices");
    }

    display.display();
    // Serial.println("");
}

// ==== SENSORS ====

bool setup_sensors() {
    display.clear();
    display.drawString(0, 0, "1-Wire Searching...");
    display.display();
    Serial.println("1-Wire Searching...");

    numberOfDevices = findDevices();
    if (!numberOfDevices) {
        // should this just be return? or should it be done during setup?
        return false;
    }

    xTaskCreate(taskUpdateDevices, /* Function to implement the task */
                "updateDevices ",  /* Name of the task */
                4000,              /* Stack size in words */
                NULL,              /* Task input parameter */
                5,                 /* Priority of the task */
                NULL);             /* Task handle. */

  // Start Device Reading Task pinned on second core
  // xTaskCreatePinnedToCore(
  //     taskUpdateDevices, /* Function to implement the task */
  //     "updateDevices ",    /* Name of the task */
  //     4000,              /* Stack size in words */
  //     NULL,              /* Task input parameter */
  //     5,                 /* Priority of the task */
  //     NULL,              /* Task handle. */
  //     1);                /* Core where the task should run */

    return true;
}

void loop_sensors() {
    for (int d = 0; d < numberOfDevices; d++) {
        OneWireDevice *device = devices[d];
        device->update();
    }

    delay(SENSOR_SAMPLE_SECONDS * 1000);
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
    while (true) {
        loop_sensors();
    } 
}

// ==== WiFi ====

bool setup_wifi() { 
    display.clear();
    display.drawString(0, 0, "WiFi Searching...");
    display.display();

    Serial.println("WiFi Searching...");
   return wifi_verify(WIFI_CHECK_TIMEOUT_SECONDS);
}

void loop_wifi() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi Searching...");
        wifi_verify(WIFI_CHECK_TIMEOUT_SECONDS);
        Serial.println("");
    }
}

// ==== MQTT ====

bool setup_mqtt() {
  if (WiFi.status() == WL_CONNECTED) {
    //   display.clear();
    //   display.drawString(0, 0, "MQTT Connecting...");
    //   display.display();

      Serial.print("MQTT Connecting...");
      mqtt.setServer(MQTT_SERVER, MQTT_PORT);
      while (!mqtt.connected()) {
          if (mqtt.connect(MQTT_CLIENTID, MQTT_USERNAME, MQTT_PASSWORD)) {
            Serial.println("connected");
            // String topic = String(MQTT_TOPIC) + String("/birth");
            // String payload = "{ \"message\": \"It's alive!\" }";
            // mqtt.publish(topic.c_str(), payload.c_str());
            return true;
          }
      }
  }
}

void loop_mqtt() {
    static uint32_t lastPublish = 0;
    if (((millis() - lastPublish) >= (1000 * MQTT_PUBLISH_INTERVAL_SECONDS))) {
        if (!mqtt.connected()) {
            setup_mqtt();
        }

        if (mqtt.connected()) {
            Serial.println("MQTT Publish...");
            for (byte d = 0; d < numberOfDevices; d++) {
                OneWireDevice *device = devices[d];
                String topic = String(MQTT_TOPIC) + String("/") + String(device->getAddressString());
                String payload = device->toJSON();

                mqtt.publish(topic.c_str(), payload.c_str());

                Serial.print(device->toString());
                Serial.print(" ==> ");
                Serial.print(topic);
                Serial.print(": ");
                Serial.println(payload);

                device->reset();
            }
        }

        lastPublish = millis();
    }
}
