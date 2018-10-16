#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define ARRAY_SIZE 5
#define Address_byte 0x20
#define GPIOA 0x12
#define IODIRA 0x00 // I/O DIRECTION REGISTER A
#define Output_mode  0x00
#define WiFi_Connect 0xFB   // Blue Color
#define CS D4

byte outputs = 0x07;
String myArray[ARRAY_SIZE];
String inp = "";
int counter = 0;

LiquidCrystal_I2C lcd(0x27,16,2);

void setup() {
  
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();
  
  WriteOutput(Address_byte,IODIRA,Output_mode); // setup output port
  WriteOutput(Address_byte,GPIOA,outputs); // setup output port

  pinMode(CS, OUTPUT);
  pinMode(D3, OUTPUT);
  digitalWrite(D3,HIGH);
  outputs = (outputs | 0x07) & WiFi_Connect;
  WriteOutput(Address_byte,GPIOA,outputs);

  
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available() >0){
    inp = Serial.readString();
    Serial.println("Gotcha");
    if(counter != ARRAY_SIZE){
      myArray[counter] = inp;
      counter++;
      Serial.println(counter);
      for(int i = 0; i<ARRAY_SIZE;i++){
        Serial.print(myArray[i]);
      }
    }
     else{
      Serial.print("Time FULL");
      outputs = outputs | 0x78;
      WriteOutput(Address_byte,GPIOA,outputs);
       if(inp.equals("clear")){
        for(int i = 0; i < ARRAY_SIZE; i++){
          myArray.remove(i);
        }
        outputs = outputs | 0x87;
        WriteOutput(Address_byte,GPIOA,outputs);
      }
      
     }
  }
}

void WriteOutput(byte ControlByte, byte RegisterAddress , byte Value )
{
  Wire.beginTransmission(ControlByte);  //Send ID Code
  Wire.write(RegisterAddress); //Send Data
  Wire.write(Value);
  Wire.endTransmission(); //Stop Condition
}
