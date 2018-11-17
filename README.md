OneWire Weather Station over LoRa

## Dependencies

* [Driver for the SSD1306 and SH1106](https://github.com/ThingPulse/esp8266-oled-ssd1306)
* [OneWire - PJRC](https://www.pjrc.com/teensy/td_libs_OneWire.html)

## Notes and References

AAGElectronica/Dallas OneWire Weather Station

* DS2450 Quad A/D
* DS2423 RAM/Counter
* DS1820 Temperature


esp32 oled LoRa Wifi -- https://www.amazon.com/gp/product/B0781CPHT1/ref=oh_aui_detailpage_o02_s00?ie=UTF8&psc=1

esp32 Install for Arduino -- https://github.com/espressif/arduino-esp32/blob/master/docs/arduino-ide/mac.md

Old Meets New; 1-Wire on SPARK Core -- https://www.element14.com/community/groups/internet-of-things/blog/2015/05/13/old-meets-new-the-1-wire-weather-station-on-the-spark-core-part-7

OneWire and WiFi -- https://github.com/espressif/arduino-esp32/issues/755

OLED -- https://github.com/Heltec-Aaron-Lee/WiFi_Kit_series/tree/master/esp32/libraries/OLED

OLED WxStation Notes -- https://github.com/osresearch/esp32-ttgo

Basis for OneWire classes -- https://github.com/jbechter
Much more extensive One Wire temperature library -- https://github.com/milesburton/Arduino-Temperature-Control-Library

```
#include <Wire.h>
#include <SSD1306.h> 
#include <OLEDDisplayUi.h>

// OLED pins to ESP32 GPIOs via this connecthin:
#define OLED_ADDRESS 0x3c
#define OLED_SDA 4 // GPIO4
#define OLED_SCL 15 // GPIO15
#define OLED_RST 16 // GPIO16

SSD1306 display(OLED_ADDRESS, OLED_SDA, OLED_SCL);
OLEDDisplayUi ui( &display );

void setup()
{
	pinMode(OLED_RST,OUTPUT);
	digitalWrite(OLED_RST, LOW); // low to reset OLED
	delay(50); 
	digitalWrite(OLED_RST, HIGH); // must be high to turn on OLED
	// ....
}
```

OneWire via Weather Toys -- http://www.weathertoys.net/weathertoys/downloads.html

Heltec ESP32 WiFi Lora -- https://robotzero.one/heltec-wifi-kit-32/



weather.c in oww
```
  if (ws.gust_end_time > ws.gust_start_time)
    {
      /* calculate the wind speed based on the revolutions per second */
      revolution_sec = (((double)(ws.gust_end_count - ws.gust_start_count) * 1000.0) /
                        (double) (ws.gust_end_time - ws.gust_start_time)) / 2.0;
      if (revolution_sec >= 0)
      {
        if ((revolution_sec >= 0)
            && (revolution_sec * 2.453 < 200.0))
        {
          ws.anem_speed =
            (float) revolution_sec
            *2.453F;

          /* Allow for calibration */
          ws.anem_speed = ws.anem_speed *
                          devices_list[devices_anem].
                          calib[0];

          if (ws.anem_speed > ws.anem_speed_max)
            ws.anem_speed_max =
              ws.anem_speed;
          if (ws.anem_speed > ws.anem_int_gust_scratch)
            ws.anem_int_gust_scratch =
              ws.anem_speed;
          //werr(WERR_DEBUG0, "%f", ws.anem_speed) ;
        }
        else
...
 /* Convert mph to m/s */
    ws.anem_mps = ws.anem_speed * 0.447040972F;

```
