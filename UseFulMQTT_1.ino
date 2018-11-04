#include <DNSServer.h>
#include "SHT21.h"
#include <Wire.h>
#include <WiFiManager.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#define SensorID "CU_A001"
#define MQTT_Port 1883
#define mqtt_server "cusensor.net" // 164.115.27.177
#define mqtt_user "nansensor"
#define mqtt_pass "nan1357"
#define CS D4

#define Address_byte 0x20  // control byte not include R/W bit use just 7 bit
#define Output_mode  0x00

#define GPIOA  0x12
#define IODIRA 0x00 // I/O DIRECTION REGISTER A

#define WiFi_Disconnect 0xFE   // Red Color
#define WiFi_Connect 0xF8   // White Color
#define Server_Disconnect 0xFB // Blue Color
#define Server_Connect 0xFD    // Green Color

#define Publish_Topic  "SmartFarm/"
#define Subscribe_Topic  "SmartFarmControl/"
/*------ Variable part ------*/

// Update these with values suitable for your network.
//const char* ssid     = "true_home2G_cbb";//"TP-LINK_A3B4";//"TESR IoT WiFi";
const char* password = "73835419";//"02656359"; //"0904656519";

byte outputs = 0x07;

String Relay_Status1 = "OFF", Relay_Status2 = "OFF", Relay_Status3 = "OFF", Relay_Status4 = "OFF";

bool shouldSaveConfig = false;

const int channelID = 608056;
String writeAPIKey = "6Q9A79FXZ7S4C44E"; // write API key for your ThingSpeak Channel
const char* server = "api.thingspeak.com";

WiFiClient espClient;
WiFiClient aclient;
PubSubClient client(espClient);
SHT21 SHT21;
WiFiManager wifiManager;

void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  Serial.println(myWiFiManager->getConfigPortalSSID());
}

void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void callback(char* topic, byte* payload, unsigned int msglen) 
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  char strState[msglen];
  for (int i=0;i<msglen;i++) 
  {
    strState[i] = (char)payload[i];
    Serial.print((char)payload[i]);
  }
  Serial.println();

  String stateStr = String(strState).substring(0, msglen);
  Serial.println(stateStr);
  if(stateStr == "ON1")
  {
    Serial.println("Command ON");
    outputs = outputs | 0x08;
    WriteOutput(Address_byte,GPIOA,outputs); 
    Relay_Status1 = "ON";
  }
  else if(stateStr == "OFF1")
  {
    Serial.println("Command OFF");
    outputs = outputs & 0xF7;
    WriteOutput(Address_byte,GPIOA,outputs); 
    Relay_Status1 = "OFF";
  }
  else if(stateStr == "ON2")
  {
    outputs = outputs | 0x10;
    WriteOutput(Address_byte,GPIOA,outputs); 
    Relay_Status2 = "ON";
    
  }
  else if(stateStr == "OFF2")
  {
    outputs = outputs & 0xEF;
    WriteOutput(Address_byte,GPIOA,outputs); 
    Relay_Status2 = "OFF";
  }
  else if(stateStr == "ON3")
  {
    outputs = outputs | 0x20;
    WriteOutput(Address_byte,GPIOA,outputs); 
    Relay_Status3 = "ON";
  }
  else if(stateStr == "OFF3")
  {
    outputs = outputs & 0xDF;
    WriteOutput(Address_byte,GPIOA,outputs); 
    Relay_Status3 = "OFF";
  }
  else if(stateStr == "ON4")
  {
    outputs = outputs | 0x40;
    WriteOutput(Address_byte,GPIOA,outputs); 
    Relay_Status4 = "ON";
  }
  else if(stateStr == "OFF4")
  {
    outputs = outputs & 0xBF;
    WriteOutput(Address_byte,GPIOA,outputs); 
    Relay_Status4 = "OFF";
  }
  else if(stateStr == "ONALL")
  {
    outputs = outputs | 0x78;
    WriteOutput(Address_byte,GPIOA,outputs);
    Relay_Status1 = "ON";
    Relay_Status2 = "ON";
    Relay_Status3 = "ON"; 
    Relay_Status4 = "ON";
  }
  else if(stateStr == "OFFALL")
  {
    outputs = outputs & 0x87;
    WriteOutput(Address_byte,GPIOA,outputs);
    Relay_Status1 = "OFF";
    Relay_Status2 = "OFF";
    Relay_Status3 = "OFF"; 
    Relay_Status4 = "OFF";
  }
}

void reconnect() 
{
  // Loop until we're reconnected
  while (!client.connected()) 
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(SensorID,mqtt_user,mqtt_pass))
    {
      Serial.println("connected");
      outputs = (outputs | 0x07) & Server_Connect;
      WriteOutput(Address_byte,GPIOA,outputs);
      // Once connected, publish an announcement...
      //client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe(Subscribe_Topic);
    } 
    else 
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup()
{
  pinMode(CS, OUTPUT); 
  SHT21.begin();
  Serial.begin(115200);
  Serial.println("Starting...");
  WriteOutput(Address_byte,IODIRA,Output_mode); // setup output port
  WriteOutput(Address_byte,GPIOA,outputs); // setup output port
  

  
  //WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) 
  {
    WiFiManager wifiManager;
    wifiManager.setAPCallback(configModeCallback);
    wifiManager.setSaveConfigCallback(saveConfigCallback);
    delay(250);
    Serial.print(".");
    outputs = (outputs | 0x07) & WiFi_Disconnect;
    WriteOutput(Address_byte,GPIOA,outputs);
  }
  outputs = (outputs | 0x07) & WiFi_Connect;
  WriteOutput(Address_byte,GPIOA,outputs);

  Serial.println("WiFi connected");  
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  client.setServer(mqtt_server, MQTT_Port);
  client.setCallback(callback);
  client.subscribe(Subscribe_Topic);
}

void loop()
{
  if (client.connected()){
    Serial.print("Humidity(%RH): ");
    Serial.print(SHT21.getHumidity());
    Serial.print("     Temperature(C): ");
    Serial.println(SHT21.getTemperature());
    char Payload[100];
    String Data = "{\"Humidity\":" + String(SHT21.getHumidity()) + 
    ",\"Temp\":" + String(SHT21.getTemperature()) + "}" ;
    Data.toCharArray(Payload, sizeof(Payload));
    String topicx = Publish_Topic + (String)SensorID;
    client.publish(topicx.c_str(), Payload);
    delay(1000);
  }
  else
     {
      outputs = (outputs | 0x07) & Server_Disconnect;
      WriteOutput(Address_byte,GPIOA,outputs);
      Serial.println("connection lost, reconnect...");
      reconnect();
      delay(1000);
     }
  if(WiFi.status() != WL_CONNECTED) 
  {
    WiFiManager wifiManager;
    wifiManager.setAPCallback(configModeCallback);
    wifiManager.setSaveConfigCallback(saveConfigCallback);
    delay(250);
    Serial.print(".");
    outputs = (outputs | 0x07) & WiFi_Disconnect;
    WriteOutput(Address_byte,GPIOA,outputs);
  }

  if (aclient.connect(server, 80)) {
    
    // Measure Signal Strength (RSSI) of Wi-Fi connection
    long temp = SHT21.getTemperature();
    long hum = SHT21.getHumidity();

    // Construct API request body
    String body = "field1=";
           body += String(temp);
           body += "&field2=";
           body += String(hum);
    
    Serial.print("RSSI: ");
    Serial.println(temp); 

    aclient.println("POST /update HTTP/1.1");
    aclient.println("Host: api.thingspeak.com");
    aclient.println("User-Agent: ESP8266 (nothans)/1.0");
    aclient.println("Connection: close");
    aclient.println("X-THINGSPEAKAPIKEY: " + writeAPIKey);
    aclient.println("Content-Type: application/x-www-form-urlencoded");
    aclient.println("Content-Length: " + String(body.length()));
    aclient.println("");
    aclient.print(body);

  }
  aclient.stop();
  delay(500);

  
  
}

void WriteOutput(byte ControlByte, byte RegisterAddress , byte Value )
{
  Wire.beginTransmission(ControlByte);  //Send ID Code
  Wire.write(RegisterAddress); //Send Data
  Wire.write(Value);
  Wire.endTransmission(); //Stop Condition
}
