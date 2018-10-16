#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <SPI.h> // include the SPI library:
#include <Wire.h> // include the I2C library:

/*------ define part ------*/
#define SCK D5
#define MISO D6
#define MOSI D7
#define CS D4

/************** define register part ****************/
#define IODIRA 0x00 // I/O DIRECTION REGISTER A
#define IODIRB 0x01 // I/O DIRECTION REGISTER B
#define GPIOA  0x12 // Set/Reset GPIOA REGISTER
#define GPIOB  0x13 // Set/Reset GPIOA REGISTER
#define GPPUA  0x0C // GPIO PULL-UP RESISTOR REGISTER
#define GPPUB  0x0D // GPIO PULL-UP RESISTOR REGISTER

/************* define command part ********************/
#define Address_byte 0x20  // control byte not include R/W bit use just 7 bit
#define Output_mode  0x00
#define Intput_mode  0xFF

#define WiFi_Disconnect 0xFE   // Red Color
#define WiFi_Connect 0xF8   // White Color
#define Server_Disconnect 0xFB // Blue Color
#define Server_Connect 0xFD    // Green Color

#define SensorID "CU_A001"
#define MQTT_Port 1883
#define mqtt_server "cusensor.net" // 164.115.27.177
#define mqtt_user "nansensor"
#define mqtt_pass "nan1357"

#define Publish_Topic  "SmartFarm/"
#define Subscribe_Topic  "SmartFarmControl/"
/*------ Variable part ------*/

// Update these with values suitable for your network.
const char* ssid     = "TP-LINK_A3B4";//"TESR IoT WiFi";
const char* password = "02656359"; //"0904656519";

unsigned long ADC_DATA = 0;
unsigned long ADC_average = 0; 
unsigned long ADC_cal = 0; 
char UART_string[150];
byte outputs = 0x07;

int count = 1;
int target_count = 50;

unsigned long Sampling_previousMillis = 0;        // will store last time LED was updated
const long Sampling_interval = 1000;           // interval at which to blink (milliseconds)

unsigned long UpdateData_previousMillis = 0;        // will store last time LED was updated
const long UpdateData_interval = 1000;           // interval at which to blink (milliseconds)

String Relay_Status1 = "OFF", Relay_Status2 = "OFF", Relay_Status3 = "OFF", Relay_Status4 = "OFF";

/*------ Prototype function part ------*/
int read_adc(int channel_x);
void WriteOutput(byte ControlByte, byte RegisterAddress , byte Value);
byte ReadInput(byte ControlByte, byte RegisterAddress);

WiFiClient espClient;
PubSubClient client(espClient);

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
  //set pin modes 
  pinMode(CS, OUTPUT); 
  SPI.begin();
  SPI.setDataMode( SPI_MODE0 ); // set SPI Mode 0,0
  SPI.setBitOrder( MSBFIRST  ); // send MSB first
  SPI.setClockDivider( SPI_CLOCK_DIV32 ); // set SCK freq. = 16MHz/32 = 500kHz
  
  Serial.begin(115200);
  Serial.println("Starting...");

  Wire.begin();
  WriteOutput(Address_byte,IODIRA,Output_mode); // setup output port
  WriteOutput(Address_byte,GPIOA,outputs); // setup output port

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) 
  {
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
  unsigned long currentMillis = millis();
  if(currentMillis - Sampling_previousMillis >= Sampling_interval) 
  {
    ADC_DATA = read_adc(0);
    //Serial.println(ADC_DATA);
    if(count <= target_count)
    {
      count++;
      ADC_cal += ADC_DATA;
      //Serial.println(ADC_average);
    }
    else
    {
      ADC_average = ADC_cal/target_count;
      Serial.println(ADC_average);
      count = 1;
      //sprintf(UART_string,"CH0 = %d , CH1 = %d  , CH2 = %d  , CH3 = %d  , CH4 = %d  , CH5 = %d  , CH6 = %d  , CH7 = %d  ",ADC_DATA[0],ADC_DATA[1],ADC_DATA[2],ADC_DATA[3],ADC_DATA[4],ADC_DATA[5],ADC_DATA[6],ADC_DATA[7]);
      //Serial.println(UART_string);
      ADC_cal = 0;
    }
    Sampling_previousMillis = currentMillis;
  }

  if (currentMillis - UpdateData_previousMillis >= UpdateData_interval) 
  {
    if (client.connected()) 
    {
      outputs = (outputs | 0x07) & Server_Connect;
      WriteOutput(Address_byte,GPIOA,outputs);
      client.loop();
      Serial.println("connect...");
      char Payload[100];
      String Data = 
      "{\"Level\":" + String(ADC_average) + 
      ",\"ID\":" + SensorID + 
      ",\"Relay1\":" + String(Relay_Status1) +
      ",\"Relay2\":" + String(Relay_Status2) +
      ",\"Relay3\":" + String(Relay_Status3) +
      ",\"Relay4\":" + String(Relay_Status4) +
      "}" ;
      Data.toCharArray(Payload, sizeof(Payload));
      String topicx = Publish_Topic + (String)SensorID;
      client.publish(topicx.c_str(), Payload);
      UpdateData_previousMillis = currentMillis;
     }
     else
     {
      outputs = (outputs | 0x07) & Server_Disconnect;
      WriteOutput(Address_byte,GPIOA,outputs);
      Serial.println("connection lost, reconnect...");
      reconnect();
     }
  }
}
 
int read_adc(int channel_x)
{
  int adcvalue = 0; // for return value
  byte buf_data[3];
  byte channel = 0;
  byte CH0 = 0xC0 , CH1 = 0xC8 , CH2 = 0xD0 , CH3 = 0xD8 
     , CH4 = 0xE0 , CH5 = 0xE8 , CH6 = 0xF0 , CH7 = 0xF8; //command bits - Start,Single,D2,D1,D0,0,0,0
     
  switch(channel_x) //Channel?
  {
    case 0: //CH0
    channel = CH0;
    break;
    case 1: //CH1
    channel = CH1;
    break;
    case 2: //CH2
    channel = CH2;
    break;
    case 3: //CH3
    channel = CH3;
    break;
    case 4: //CH4
    channel = CH4;
    break;
    case 5: //CH5
    channel = CH5;
    break;
    case 6: //CH6
    channel = CH6;
    break;
    case 7: //CH7
    channel = CH7;
    break;
    default: // protection case
    channel = CH0;
  }

  digitalWrite(CS,LOW); // Enable MCP3208
  buf_data[0] = SPI.transfer(channel);
  buf_data[1] = SPI.transfer(0x00);
  buf_data[2] = SPI.transfer(0x00);
  digitalWrite(CS, HIGH); // Disable MCP3208
  adcvalue = ((buf_data[0] & 0x01)<<11) + (buf_data[1]<<3) + ((buf_data[2]&0xE0)>>5);
  return adcvalue;
}

void WriteOutput(byte ControlByte, byte RegisterAddress , byte Value )
{
  Wire.beginTransmission(ControlByte);  //Send ID Code
  Wire.write(RegisterAddress); //Send Data
  Wire.write(Value);
  Wire.endTransmission(); //Stop Condition
}

byte ReadInput(byte ControlByte, byte RegisterAddress)
{
  Wire.beginTransmission(ControlByte);
  Wire.write(RegisterAddress);
  Wire.endTransmission();
  Wire.requestFrom(ControlByte, 1);
  return(Wire.read());
}
