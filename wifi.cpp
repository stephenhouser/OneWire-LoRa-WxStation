#include <Arduino.h>
#include <WiFi.h>

#include "esp_wpa2.h"
#define WL_MAC_ADDR_LENGTH 6

struct WiFiNet {
  const char *ssid;
  const char *username;
  const char *password;
};

#include "credentials.h"
/* PREDEFINED_NETWORKS is a #define in credentials.h that defines
 * preconfigured wireless networks to try and connect to (in order).
 * The code below handles both WPA2 personal and WPA2 enterprise.
 *
 * To specify a WPA2 personal network, set the "username" to NULL.
 * To specify a WPA2 enterprise network, set all three fields.
 *
 * It should look something very much like this.
 * #define PREDEFINED_NETWORKS { {"SSID", "username", "password"}, ...}
 */
const struct WiFiNet known_networks[] = PREDEFINED_NETWORKS;

const struct WiFiNet *find_ssid(const char *ssid) {
  int known_networks_count = sizeof(known_networks) / sizeof(struct WiFiNet);
  // Serial.printf("%d known networks\n", known_networks_count);
  for (int i = 0; i < known_networks_count; i++) {
    if (!strcmp(known_networks[i].ssid, ssid)) {
      return &known_networks[i];
    }
  }

  return NULL;
}

void wifi_connect(const char *ssid, const char *username,
                  const char *password) {
  if (username == NULL) {
    WiFi.begin(ssid, password);
  } else {
    WiFi.disconnect(true); // disconnect form wifi to set new wifi connection
    WiFi.mode(WIFI_STA);
    esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)username,
                                       strlen(username)); // provide identity
    esp_wifi_sta_wpa2_ent_set_username((uint8_t *)username,
                                       strlen(username)); // provide username
    esp_wifi_sta_wpa2_ent_set_password((uint8_t *)password,
                                       strlen(password)); // provide password
    esp_wpa2_config_t config = WPA2_CONFIG_INIT_DEFAULT(); // set config to default (fixed for 2018 and
                                    // Arduino 1.8.5+)
    esp_wifi_sta_wpa2_ent_enable(&config); // set config to enable function
                                           // (fixed for 2018 and Arduino 1.8.5+)
    WiFi.begin(ssid);
    WiFi.setHostname("OneWireLoRA"); // set Hostname for your device - not neccesary
  }
}

const struct WiFiNet *wifi_scan() {
  Serial.printf("scanning wifi... %d known networks, ",
                sizeof(known_networks) / sizeof(struct WiFiNet));
  const struct WiFiNet *best_ap = NULL;
  int best_signal = -100;

  // WiFi.scanNetworks will return the number of networks found
  int networks = WiFi.scanNetworks();
  if (networks != 0) {
    Serial.printf("%d networks found.\n", networks);

    for (int i = 0; i < networks; ++i) {
      Serial.printf("%d: %s, Ch:%d (%ddBm) %s\n", i + 1, WiFi.SSID(i).c_str(),
                    WiFi.channel(i), WiFi.RSSI(i),
                    WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? "open" : "");

      if (WiFi.RSSI(i) > best_signal) {
        const struct WiFiNet *net = find_ssid(WiFi.SSID(i).c_str());
        if (net != NULL) {
          best_ap = net;
          best_signal = WiFi.RSSI(i);
        }
      }
    }
  }

  if (best_ap) {
    Serial.printf("Using [%s, %ddBm]\n", best_ap->ssid, best_signal);
  } else {
    Serial.println("Did not find acceptable network");
  }
  return best_ap;
}

void setupAP() {
  Serial.println("\nFailed to setup as client");
  WiFi.disconnect(true);
  WiFi.mode(WIFI_AP);
  Serial.println("Configuring access point...");

  // Do a little work to get a unique-ish name.
  uint8_t mac[WL_MAC_ADDR_LENGTH];
  WiFi.softAPmacAddress(mac);
  String AP_NameString = "ESP32_" + String(mac[WL_MAC_ADDR_LENGTH - 2], HEX) +
                         String(mac[WL_MAC_ADDR_LENGTH - 1], HEX);
  AP_NameString.toUpperCase();
  char AP_NameChar[AP_NameString.length() + 1];
  memset(AP_NameChar, 0, AP_NameString.length() + 1);
  for (int i = 0; i < AP_NameString.length(); i++) {
    AP_NameChar[i] = AP_NameString.charAt(i);
  }

  WiFi.softAP(AP_NameChar, "12345678");
}

bool wifi_try_connect(const char *ssid, const char *username, const char *password, int timeout = 15) {
    Serial.print("connecting ");
    Serial.print(ssid);
    wifi_connect(ssid, username, password);
    int seconds = 0;
    while (WiFi.status() != WL_CONNECTED && seconds <= timeout) {
        seconds++; // increment try wait
        delay(1000);
        Serial.print(".");
    }
}

bool wifi_try_known_networks() {
  int known_networks_count = sizeof(known_networks) / sizeof(struct WiFiNet);
  // Serial.printf("%d known networks\n", known_networks_count);
  for (int i = 0; i < known_networks_count; i++) {
    const struct WiFiNet *net = &known_networks[i];
    if (wifi_try_connect(net->ssid, net->username, net->password)) {
      return true;
    }
  }

  return false;
}

bool wifi_verify(int timeout) {
  if (WiFi.status() != WL_CONNECTED) {
    // Scan for a known network
    const struct WiFiNet *ap = wifi_scan();
    if (ap != NULL) {
        wifi_try_connect(ap->ssid, ap->username, ap->password);
    } else {
        wifi_try_known_networks();
    }
  }

//   // we have a connection or we have tried 10 times
//   // if we don't then setup an AP
//   if (WiFi.status() != WL_CONNECTED) {
//     setupAP();
//   } else {
//     // Now we are connected
//     Serial.print(" connected IP=");
//     Serial.println(WiFi.localIP());
//     // configTime(3 * 3600, 0, "pool.ntp.org", "time.nist.gov");
//     // Serial.println("done.");
//   }

  return true;
}
