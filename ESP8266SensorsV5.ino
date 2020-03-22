#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#define WLAN_SSID       "534887"
#define WLAN_PASS       "262897820"
#define MQTT_SERVER      "192.168.0.30" // static ip address
#define MQTT_PORT         1883
#define MQTT_USERNAME    ""
#define MQTT_PASSWORD         ""
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, MQTT_PORT, MQTT_USERNAME, MQTT_PASSWORD);
Adafruit_MQTT_Publish sendToPi = Adafruit_MQTT_Publish(&mqtt, MQTT_USERNAME "/espdata");
Adafruit_MQTT_Subscribe esp8266_led = Adafruit_MQTT_Subscribe(&mqtt, MQTT_USERNAME "/leds/esp8266");

uint32_t x = 0;
void MQTT_connect();

char piSend[20] = "";

void setup()
{
  Serial.println(F("RPi-ESP-MQTT"));
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  Serial.begin(115200);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }
  Serial.println("");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void loop()
{
  String IncomingString = "";
  boolean StringReady = false;
  MQTT_connect();

  while (Serial.available()) {
    IncomingString = Serial.readString();
    StringReady = true;
    IncomingString.toCharArray(piSend, 20);
    sendToPi.publish(piSend);
  }
}

void MQTT_connect() {
  int8_t ret;
  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }
  Serial.print("Connecting to MQTT... ");
  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0) {
      // basically die and wait for WDT to reset me
      while (1);
    }
  }
  Serial.println("MQTT Connected!");
}
