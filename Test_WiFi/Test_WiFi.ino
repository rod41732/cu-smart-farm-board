#include <ESP8266WiFi.h>
#include <PubSubClient.h>

//SSID of your network
const char* ssid = "pewt1403"; //SSID of your Wi-Fi router
const char* pass = "aabbabab"; //Password of your Wi-Fi router
const char* mqtt_server = "";

WiFiClient espClient;
PubSubClient client(espClient);

void setup()
{
  Serial.begin(115200);
  delay(10);

  // Connect to Wi-Fi network
  Serial.println();
  Serial.println();
  Serial.print("Connecting to...");
  Serial.println(ssid);

  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("Wi-Fi connected successfully");
}

void loop () {}
