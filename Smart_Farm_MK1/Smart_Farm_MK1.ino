#include <ESP8266WiFi.h>
#include <MicroGear.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h> // inslude the SPI library:
#include <LoRa.h>
#include <time.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <PubSubClient.h>

/*------ define part ------*/
#define SCK D5
#define MISO D6
#define MOSI D7
#define CS D4
#define ARRAY_SIZE 5

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

#define OLED_RESET -1

Adafruit_SSD1306 OLED(OLED_RESET);
#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

/*------ Variable part ------*/
int ADC_Value = 0; 
int ADC_DATA[8]; 
char UART_string[150];
byte outputs = 0x07;

unsigned long previousMillis_NETPIE = 0 , previousMillis_LCD = 0 , previousMillis_Sampling = 0;        // will store last time LED was updated
const long interval_LCD = 1000;                                          // interval at which to blink (milliseconds)
const long interval_NETPIE = 250;
const long interval_Sampling = 100;

const char* ssid     = "Pewt";//"TP-LINK_A3B4"
const char* password = "aabbabab";//"02656359"
const char* mqtt_server = ""

String Relay_Status1, Relay_Status2, Relay_Status3, Relay_Status4;
String Relay_Mode1 , Relay_Mode2 , Relay_Mode3 , Relay_Mode4;
String TimeNow;

const char* Server1 = "ntp.ku.ac.th";
const char* Server2 = "time.navy.mi.th";
const char* Server3 = "clock.nectec.or.th";

int Hour_Now_Times = 0, Min_Now_Times = 0, Sec_Now_Times = 0;
int Hour_On_Times = 12, Min_On_Times = 10, Sec_On_Times = 5;
int Hour_Off_Times = 12, Min_Off_Times = 45, Sec_Off_Times = 30; // Unit min
int Soil_Moisture_Threshold = 120; // Unit ADC
int Humidity_Threshold = 60 , Temp_Threshold = 37; // Unit % , C

float temparature = 0 , humidity = 0;
int adc_Soil_Moisture = 0;

float temparature_previous = 0 , humidity_previous = 0;
int adc_Soil_Moisture_previous = 0;

float temparature_array[10], humidity_array[10];
int adc_Soil_Moisture_array[10];

int index_count = 0;
bool Time_on = false;

int timezone = 7 * 3600;
int dst = 0;

/*------ Prototype function part ------*/
int read_adc(int channel_x);
void WriteOutput(byte ControlByte, byte RegisterAddress , byte Value);
byte ReadInput(byte ControlByte, byte RegisterAddress);
String zero2digit(int num);

// Pin Connect D1(SCL) , D2(SDA) VCC 5V
LiquidCrystal_I2C lcd(0x27,16,2); 

WiFiClient client;
AuthClient *authclient;

MicroGear microgear(client);

void onMsghandler(char *topic, uint8_t* msg, unsigned int msglen) 
{
  Serial.print("Incoming message --> ");
  Serial.print(topic);
  Serial.print(" : ");
  char strState[msglen];
  for (int i = 0; i < msglen; i++) 
  {
    strState[i] = (char)msg[i];
    Serial.print((char)msg[i]);
  }
  Serial.println();

  String stateStr = String(strState).substring(0, msglen);
  if(stateStr == "ON1")
  {
    outputs = outputs | 0x08;
    WriteOutput(Address_byte,GPIOA,outputs); 
    Relay_Status1 = "ON";
  }
  else if(stateStr == "OFF1")
  {
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
  else if(stateStr == "MA1")
  {
    Relay_Mode1 = "Auto";
  }
  else if(stateStr == "MM1")
  {
    Relay_Mode1 = "Manual";
  }
  else if(stateStr == "MA2")
  {
    Relay_Mode2 = "Auto";
  }
  else if(stateStr == "MM2")
  {
    Relay_Mode2 = "Manual";
  }
  else if(stateStr == "MA3")
  {
    Relay_Mode3 = "Auto";
  }
  else if(stateStr == "MM3")
  {
    Relay_Mode3 = "Manual";
  }
  else if(stateStr == "MA4")
  {
    Relay_Mode4 = "Auto";
  }
  else if(stateStr == "MM4")
  {
    Relay_Mode4 = "Manual";
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
  else if(stateStr.indexOf('=')) // example T=100
  {
    int split_index = stateStr.indexOf('=');
    String command = stateStr.substring(0, split_index);
    String value = stateStr.substring(split_index+1);
    if(command == "S") // Soil_Moisture_Threshold
    {
      Soil_Moisture_Threshold = value.toInt();
    }
    else if(command == "H") // Himidity
    {
      Humidity_Threshold = value.toInt();
    }
    else if(command == "C") // Temp
    {
      Temp_Threshold = value.toInt();
    }
    else if(command == "T_on_h") // Time on hour
    {
      Hour_On_Times = value.toInt();
    }
    else if(command == "T_on_m") // Time on min
    {
      Min_On_Times = value.toInt();
    }
    else if(command == "T_on_s") // Time on sec
    {
      Sec_On_Times = value.toInt();
    }
    else if(command == "T_off_h") // Time off hour
    {
      Hour_Off_Times = value.toInt();
    }
    else if(command == "T_off_m") // Time off min
    {
      Min_Off_Times = value.toInt();
    }
    else if(command == "T_off_s") // Time off sec
    {
      Sec_Off_Times = value.toInt();
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); 
  //OLED.begin(SSD1306_SWITCHCAPVCC, 0x3C);  
  Wire.begin();
  WriteOutput(Address_byte,IODIRA,Output_mode); // setup output port
  WriteOutput(Address_byte,GPIOA,outputs); // setup output port

  //set pin modes 
  pinMode(CS, OUTPUT); 
  SPI.begin();
  SPI.setDataMode( SPI_MODE0 ); // set SPI Mode 0,0
  SPI.setBitOrder( MSBFIRST  ); // send MSB first
  SPI.setClockDivider( SPI_CLOCK_DIV32 );

  pinMode(D3, OUTPUT);
  digitalWrite(D3,HIGH);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.print(".");
    outputs = (outputs | 0x07) & WiFi_Disconnect;
    WriteOutput(Address_byte,GPIOA,outputs);
  }
  
  outputs = (outputs | 0x07) & WiFi_Connect;
  WriteOutput(Address_byte,GPIOA,outputs);
  Serial.println("WiFi connected");  
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  configTime(timezone, dst, Server3,Server2,Server1);
  Serial.println("\nWaiting for Internet time");

  while(!time(nullptr)){
     Serial.print("*");
     delay(1000);
  }
  Serial.println("\nTime response....OK"); 

  //uncomment the line below if you want to reset token -->
  microgear.resetToken();
  //microgear.init(KEY,SECRET,ALIAS);
  //microgear.connect(APPID);

}

void loop() {
  // put your main code here, to run repeatedly:
  ADC_DATA[0] = read_adc(0); // Soil-Moisture 
  ADC_DATA[1] = read_adc(1); // Temp
  ADC_DATA[2] = read_adc(2); // Humidity
  ADC_DATA[3] = read_adc(3); // PH
  ADC_DATA[4] = read_adc(4); // Water
  //sprintf(UART_string,"CH0 = %d , CH1 = %d  , CH2 = %d  , CH3 = %d  , CH4 = %d  ",ADC_DATA[0],ADC_DATA[1],ADC_DATA[2],ADC_DATA[3],ADC_DATA[4]);
  //Serial.println(UART_string);
  
  adc_Soil_Moisture_previous = ADC_DATA[0];
  temparature_previous = (float)(ADC_DATA[1]*5*1000)/(4095*25.1) - 60.12956720128035 ; // 25.1 mV/°C
  humidity_previous = (float)(ADC_DATA[2]*5*1000)/(4095*44.0);      // 44.0 mV/%RH


  adc_Soil_Moisture_array[index_count] = adc_Soil_Moisture_previous;
  temparature_array[index_count] = temparature_previous;
  humidity_array[index_count] = humidity_previous;
  index_count++;
  if(index_count >= 10)
  {
    for(int count = 0; count <= 9 ; count++)
    {
      adc_Soil_Moisture += adc_Soil_Moisture_array[count];
      temparature += temparature_array[count];
      humidity += humidity_array[count];
    }
    index_count = 0;
    adc_Soil_Moisture = adc_Soil_Moisture/10;
    temparature = temparature/10;
    humidity = humidity/10;

    OLED.clearDisplay();       // clear display
    OLED.setTextColor(WHITE);  // set text color
    OLED.setTextSize(1);
    OLED.setCursor(10,0);
    String line1 = "Soil = " + String(adc_Soil_Moisture);
    OLED.println(line1);
    OLED.setCursor(10,10);
    String line2 = "Temp = " + String(temparature) + " C";
    OLED.println(line2);
    OLED.setCursor(10,20);
    String line3 = "Humid = " + String(humidity) + " %";
    OLED.println(line3);
    //OLED.setCursor(15,35);
    //OLED.println("A0 = " + String(analogRead(A0)));
    OLED.display(); 
  }
  
  unsigned long currentMillis = millis(); // update time
  /******************** Update Sensor Data ***************************/
  if (currentMillis - previousMillis_Sampling >= interval_Sampling) 
  {
    time_t now = time(nullptr);
    struct tm* p_tm = localtime(&now);
    Hour_Now_Times = int(p_tm->tm_hour);
    Min_Now_Times = int(p_tm->tm_min);
    Sec_Now_Times = int(p_tm->tm_sec);
    TimeNow = zero2digit(Hour_Now_Times) + ":" + zero2digit(Min_Now_Times) + ":" + zero2digit(Sec_Now_Times);
    Serial.println(TimeNow);
    if(Relay_Mode1 == "Auto")
    {
      if(Hour_Now_Times == Hour_On_Times && Min_Now_Times == Min_On_Times && Sec_Now_Times == Sec_On_Times)
      {
        outputs = outputs | 0x08;
        WriteOutput(Address_byte,GPIOA,outputs); 
        Relay_Status1 = "ON";
        Time_on = true;
      }
      else if(Hour_Now_Times == Hour_Off_Times && Min_Now_Times == Min_Off_Times && Sec_Now_Times == Sec_Off_Times)
      {
        outputs = outputs & 0xF7;
        WriteOutput(Address_byte,GPIOA,outputs); 
        Relay_Status1 = "OFF";
        Time_on = false;
      }

      if(Time_on == true) // ON Time
      {
        outputs = outputs | 0x08;
        WriteOutput(Address_byte,GPIOA,outputs); 
        Relay_Status1 = "ON";
      }
      else // Off Time
      {
        outputs = outputs & 0xF7;
        WriteOutput(Address_byte,GPIOA,outputs);
        Relay_Status1 = "OFF";
      }
    }
    if(Relay_Mode2 == "Auto")
    {
      if(temparature >= Temp_Threshold)
      {
        outputs = outputs | 0x10;
        WriteOutput(Address_byte,GPIOA,outputs); 
        Relay_Status2 = "ON";
      }
      else
      {
        outputs = outputs & 0xEF;
        WriteOutput(Address_byte,GPIOA,outputs); 
        Relay_Status2 = "OFF";
      }
    }
      
    if(Relay_Mode3 == "Auto")
    { 
      if(humidity <= Humidity_Threshold)
      {
        outputs = outputs | 0x20;
        WriteOutput(Address_byte,GPIOA,outputs); 
        Relay_Status3 = "ON";
      }
      else
      {
        outputs = outputs & 0xDF;
        WriteOutput(Address_byte,GPIOA,outputs); 
        Relay_Status3 = "OFF";
      }
    }
  
    if(Relay_Mode4 == "Auto")
    {
      if(adc_Soil_Moisture <= Soil_Moisture_Threshold)
      {
        outputs = outputs | 0x40;
        WriteOutput(Address_byte,GPIOA,outputs); 
        Relay_Status4 = "ON";
      }
      else
      {
        outputs = outputs & 0xBF;
        WriteOutput(Address_byte,GPIOA,outputs); 
        Relay_Status4 = "OFF";
      }
    }
    previousMillis_Sampling = currentMillis;
  }
  /*****************************************************/
  if (currentMillis - previousMillis_NETPIE >= interval_NETPIE) 
  {
    /*********Send data to NETPIE Part*********/
    if(microgear.connected()) 
    {
      microgear.loop();
      outputs = (outputs | 0x07) & Server_Connect;
      WriteOutput(Address_byte,GPIOA,outputs );
      Serial.println("connect...");
      String Message = String(adc_Soil_Moisture)+","+String(temparature)+","+String(humidity)+","+String(ADC_DATA[3])+","+String(ADC_DATA[4])+","+
      String(Relay_Status1)+","+String(Relay_Status2)+","+String(Relay_Status3)+","+String(Relay_Status4)+","+
      String(Relay_Mode1)+","+String(Relay_Mode2)+","+String(Relay_Mode3)+","+String(Relay_Mode4)+","+TimeNow;
      //microgear.chat(TARGET,Message);
      //Serial.println(Message);
    }
    else
    {
      outputs = (outputs | 0x07) & Server_Disconnect;
      WriteOutput(Address_byte,GPIOA,outputs);
      Serial.println("connection lost, reconnect...");
      //microgear.connect(APPID);
    }
    previousMillis_NETPIE = currentMillis;
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

String zero2digit(int num)
{
  String number = "";
  if(num < 10)
  {
    number = "0" + String(num);
  }
  else
  {
    number = String(num);
  }
  return number;
}
