#include "MegunoLink.h" 
#include "CommandHandler.h" 
#include <Wire.h> 
 
#define LEDPIN 13 
#define beta 3950 
#define resistance 10 

int ThermistorPin = 0; 
int Vo; 
float R1 = 2252; 
float logR2, R2, T; 
float A = 1.484778004e-03, B = 2.348962910e-04, C = 1.006037158e-07; 
unsigned long StartMillis = 0; 
float Solvent;
float TotalMillis; 
float Eflowrate;
boolean TemperatureStart = false;  
boolean TemperatureStop; 
boolean FlowStart = false; 
boolean FlowStop; 
boolean PhotoStart = false; 
boolean PhotoStop; 
const int ADDRESS = 0x08; 
const float SCALE_FACTOR_FLOW = 500.0; 
uint16_t sensor_flow_value; 
CommandHandler<> SerialCommandHandler; 
long LastSent; 
 
const unsigned SendInterval = 200; 
 
XYPlot TempPlot("Temperature Readings"), FlowPlot ("Flow rate readings"), PhotoPlot("Photochemical and temperature measurements"); 
InterfacePanel MyPanel; 
 
void Cmd_SETSYSTEM(CommandParameter &Parameters) 
{Solvent=Parameters.NextParameterAsInteger(Solvent); } 
 
void Cmd_TemperatureStart(CommandParameter &params){ 
 TemperatureStart = true;   } 
 
void Cmd_TemperatureStop(CommandParameter &params){ 
 TemperatureStart = false;   } 
 
void Cmd_FlowStart(CommandParameter &params){ 
 FlowStart = true;   } 
 
void Cmd_FlowStop(CommandParameter &params){ 
 FlowStart = false;   } 
 
void Cmd_PhotoStart(CommandParameter &params){ 
 PhotoStart = true;   } 
 
void Cmd_PhotoStop(CommandParameter &params){ 
 PhotoStart = false;   } 
 
void setup(){ 
 
Serial.begin(9600); 
Wire.begin(); 
Serial.println("MegunoLink Pro - Turning Solenoids on and off"); 
Serial.println("-----------------------------"); 
SerialCommandHandler.AddCommand(F("SETSYSTEM"), Cmd_SETSYSTEM); 
SerialCommandHandler.AddCommand(F("TemperatureStart"), Cmd_TemperatureStart); 
SerialCommandHandler.AddCommand(F("TemperatureStop"), Cmd_TemperatureStop); 
SerialCommandHandler.AddCommand(F("FlowStart"), Cmd_FlowStart); 
SerialCommandHandler.AddCommand(F("FlowStop"), Cmd_FlowStop); 
SerialCommandHandler.AddCommand(F("PhotoStart"), Cmd_PhotoStart); 
SerialCommandHandler.AddCommand(F("PhotoStop"), Cmd_PhotoStop); 

LastSent = millis(); 

TempPlot.SetSeriesProperties("ADCValue1", Plot::Red, Plot::Solid, 2, Plot::Square); 
TempPlot.SetSeriesProperties("ADCValue2", Plot::Blue, Plot::Solid, 2, Plot::Square); 
TempPlot.SetSeriesProperties("ADCValue3", Plot::Green, Plot::Solid, 2, Plot::Square); 
FlowPlot.SetSeriesProperties("Flow rate", Plot::Magenta, Plot::Solid, 5, Plot::Circle); 
PhotoPlot.SetSeriesProperties("ADCValue4", Plot::Black, Plot::Solid, 2, Plot::Triangle); 

int ret;
do {
  Wire.beginTransmission(ADDRESS);
  Wire.write(0xFE);
  ret=Wire.endTransmission();
  } while (ret !=0);
} 
 
void loop() { 
 
SerialCommandHandler.Process(); 

if ((TemperatureStart == true)&& ((millis() - LastSent) > SendInterval)) { 
  LastSent=millis(); 
  long temp1 =1023 - analogRead (A0); 
  float sensor1 = beta /(log(((1025.0 * 10 / temp1) - 10) / 10) + beta / 298.0) - 273.0; 
  long temp2 =1023 - analogRead (A1); 
  float sensor2 = beta /(log(((1025.0 * 10 / temp2) - 10) / 10) + beta / 298.0) - 273.0; 
  long temp3 =1023 - analogRead (A2); 
  float sensor3 = beta /(log(((1025.0 * 10 / temp3) - 10) / 10) + beta / 298.0) - 273.0; 
  TempPlot.SendData("ADCValue1", millis(),sensor1); 
  TempPlot.SendData("ADCValue2", millis(),sensor2);  
  TempPlot.SendData("ADCValue3", millis(),sensor3);  
  } 
if (TemperatureStop== false){ 
  } 

if ((FlowStart == true)&& ((millis() - LastSent) > SendInterval)&& (Solvent == 1)) { 
  LastSent=millis(); 
   int ret; 
    Wire.beginTransmission(ADDRESS); 
    Wire.write(0x36); 
    Wire.write(0x08); 
    ret = Wire.endTransmission(); 
    Wire.requestFrom(ADDRESS, 9); 
    sensor_flow_value  = Wire.read() << 8; 
    sensor_flow_value |= Wire.read();      
    float flow_value = ((int16_t)sensor_flow_value)/SCALE_FACTOR_FLOW;
    FlowPlot.SendData("Flow rate", millis(),flow_value); 
  }

else if ((FlowStart == true)&& ((millis() - LastSent) > SendInterval)&& (Solvent == 2)) { 
  LastSent=millis(); 
   int ret; 
    Wire.beginTransmission(ADDRESS); 
    Wire.write(0x36); 
    Wire.write(0x08); 
    ret = Wire.endTransmission(); 
    Wire.requestFrom(ADDRESS, 9); 
    sensor_flow_value  = Wire.read() << 8; 
    sensor_flow_value |= Wire.read();      
    float flow_value = ((5*(pow(10,(-5))))*(pow((int16_t)sensor_flow_value,2))+(0.0051*sensor_flow_value)+0.0215);
    FlowPlot.SendData("Flow rate", millis(),flow_value); 
  }

else if ((FlowStart == true)&& ((millis() - LastSent) > SendInterval)&& (Solvent == 3)) { 
  LastSent=millis(); 
   int ret; 
    Wire.beginTransmission(ADDRESS); 
    Wire.write(0x36); 
    Wire.write(0x08); 
    ret = Wire.endTransmission(); 
    Wire.requestFrom(ADDRESS, 9); 
    sensor_flow_value  = Wire.read() << 8; 
    sensor_flow_value |= Wire.read();      
    float flow_value = (pow(10,(-5))*(pow((int16_t)sensor_flow_value,2))+(0.0071*sensor_flow_value)-0.0918);
    FlowPlot.SendData("Flow rate", millis(),flow_value); 
    FlowPlot.SendData("sensor", millis(),sensor_flow_value); 
  }
  
  else if ((FlowStart == true)&& ((millis() - LastSent) > SendInterval)&& (Solvent == 4)) { 
  LastSent=millis(); 
   int ret; 
    Wire.beginTransmission(ADDRESS); 
    Wire.write(0x36); 
    Wire.write(0x08); 
    ret = Wire.endTransmission(); 
    Wire.requestFrom(ADDRESS, 9); 
    sensor_flow_value  = Wire.read() << 8; 
    sensor_flow_value |= Wire.read();      
    float flow_value = ((2*(pow(10,(-5))))*(pow((int16_t)sensor_flow_value,2))+(0.0085*sensor_flow_value)-0.1453);
    FlowPlot.SendData("Flow rate", millis(),flow_value);
  }
  
  else if  ((FlowStart == true)&& ((millis() - LastSent) > SendInterval)&& (Solvent == 5)) { 
  LastSent=millis(); 
   int ret; 
    Wire.beginTransmission(ADDRESS); 
    Wire.write(0x36); 
    Wire.write(0x08); 
    ret = Wire.endTransmission(); 
    Wire.requestFrom(ADDRESS, 9); 
    sensor_flow_value  = Wire.read() << 8; 
    sensor_flow_value |= Wire.read();      
    float flow_value = ((4*(pow(10,(-5))))*(pow((int16_t)sensor_flow_value,2))+(0.0054*sensor_flow_value)-0.0335);
    FlowPlot.SendData("Flow rate", millis(),flow_value); 
  }

  else if  ((FlowStart == true)&& ((millis() - LastSent) > SendInterval)&& (Solvent == 6)) { 
  LastSent=millis(); 
   int ret; 
    Wire.beginTransmission(ADDRESS); 
    Wire.write(0x36); 
    Wire.write(0x08); 
    ret = Wire.endTransmission(); 
    Wire.requestFrom(ADDRESS, 9); 
    sensor_flow_value  = Wire.read() << 8; 
    sensor_flow_value |= Wire.read();      
    float flow_value = ((3*(pow(10,(-5))))*(pow((int16_t)sensor_flow_value,2))+(0.0022*sensor_flow_value)+0.1508);
    FlowPlot.SendData("Flow rate", millis(),flow_value); 
     FlowPlot.SendData("sensor", millis(),sensor_flow_value); 
  }
 
if ((PhotoStart == true)&& ((millis() - LastSent) > SendInterval)) { 
  LastSent=millis(); 
  long temp4 =1023 - analogRead (A3); 
  float sensor4 = beta /(log(((1025.0 * 10 / temp4) - 10) / 10) + beta / 298.0) - 273.0; 
  PhotoPlot.SendData("ADCValue4", millis(),sensor4);  
  } 
if (PhotoStop== false){ 
  } 

}
