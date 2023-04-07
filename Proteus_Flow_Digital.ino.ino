#include "MegunoLink.h" 
#include "CommandHandler.h" 
#include <Wire.h> 
 
#define SolenoidAPIN 4 
#define SolenoidBPIN 5 
#define LEDPIN 13 
#define beta 3950 
#define resistance 10 

int ThermistorPin = 0; 
int Vo; 
float R1 = 2252; 
float logR2, R2, T; 
float A = 1.484778004e-03, B = 2.348962910e-04, C = 1.006037158e-07; 
unsigned long StartMillis = 0; 
float TotalMillis; 
float Pressure; 
float Capilliary; 
float InjectionVolume; 
float Solvent;
float CDRType1; 
float CDRType3; 
float LOOPA; 
float LOOPB; 
float PreCollect; 
float PostCollect; 
float FinalWash; 
float SystemVolume; 
float TotalVolume; 
float TotalTime; 
float Progress;
float Eflowrate;
boolean ReactionRUN = false; 
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
void Cmd_SolenoidAON(CommandParameter &Parameters) 
{ digitalWrite(SolenoidAPIN,HIGH); 
digitalWrite(LEDPIN, HIGH); } 
 
void Cmd_SolenoidAOFF(CommandParameter &Parameters) 
{ digitalWrite(SolenoidAPIN,LOW); 
digitalWrite(LEDPIN,LOW); } 

void Cmd_SolenoidBON(CommandParameter &Parameters) 
{ digitalWrite(SolenoidBPIN,HIGH); } 
 
void Cmd_SolenoidBOFF(CommandParameter &Parameters) 
{ digitalWrite(SolenoidBPIN,LOW);} 
 
void Cmd_ContinuousRUNON(CommandParameter &Parameters) 
{ digitalWrite(SolenoidAPIN,HIGH); 
 digitalWrite(SolenoidBPIN,HIGH);} 
 
void Cmd_ContinuousRUNOFF(CommandParameter &Parameters) 
{ digitalWrite(SolenoidAPIN,LOW); 
digitalWrite(SolenoidBPIN,LOW); 
 
MyPanel.SetProgress(("ReactionProgressBar"), 0); } 
 
void Cmd_ReactionRUN(CommandParameter &Parameters) 
{Pressure=Parameters.NextParameterAsInteger(Pressure); 
InjectionVolume=Parameters.NextParameterAsInteger(InjectionVolume); 
Capilliary=Parameters.NextParameterAsInteger(Capilliary); 
digitalWrite(SolenoidAPIN,HIGH); 
digitalWrite(SolenoidBPIN, HIGH); 
ReactionRUN = true; 
StartMillis = millis(); 
MyPanel.SetText(("TotalVolume"),(TotalVolume+InjectionVolume)); 
MyPanel.SetProgress(("ReactionProgressBar"), 0); 
MyPanel.SetText(F("Progress"), ((millis()-StartMillis)/TotalMillis)*100);
MyPanel.SetText(F("TotalTime"), (TotalTime));
MyPanel.SetText(F("Eflowrate"), (Eflowrate));
} 
 
void Cmd_SETSYSTEM(CommandParameter &Parameters) 
{CDRType1=Parameters.NextParameterAsInteger(CDRType1); 
CDRType3=Parameters.NextParameterAsInteger(CDRType3); 
LOOPA=Parameters.NextParameterAsInteger(LOOPA); 
LOOPB=Parameters.NextParameterAsInteger(LOOPB); 
PreCollect=Parameters.NextParameterAsInteger(PreCollect); 
PostCollect=Parameters.NextParameterAsInteger(PostCollect); 
FinalWash=Parameters.NextParameterAsInteger(FinalWash); 
SystemVolume=Parameters.NextParameterAsInteger(SystemVolume); 
Solvent=Parameters.NextParameterAsInteger(Solvent); 
 
if (CDRType1>=1){ 
TotalVolume=((CDRType1*3)+(PreCollect)+(PostCollect)+(SystemVolume)); } 
 
else if (CDRType3>=1){ 
TotalVolume=((CDRType3*2.8)+(PreCollect)+(PostCollect)+(SystemVolume)); }  
} 
 
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
SerialCommandHandler.AddCommand(F("SolenoidAON"), Cmd_SolenoidAON); 
SerialCommandHandler.AddCommand(F("SolenoidAOFF"), Cmd_SolenoidAOFF); 
SerialCommandHandler.AddCommand(F("SolenoidBON"), Cmd_SolenoidBON); 
SerialCommandHandler.AddCommand(F("SolenoidBOFF"), Cmd_SolenoidBOFF); 
SerialCommandHandler.AddCommand(F("ContinuousRUNON"), Cmd_ContinuousRUNON); 
SerialCommandHandler.AddCommand(F("ContinuousRUNOFF"), Cmd_ContinuousRUNOFF); 
SerialCommandHandler.AddCommand(F("SETSYSTEM"), Cmd_SETSYSTEM); 
SerialCommandHandler.AddCommand(F("ReactionRUN"), Cmd_ReactionRUN); 
SerialCommandHandler.AddCommand(F("TemperatureStart"), Cmd_TemperatureStart); 
SerialCommandHandler.AddCommand(F("TemperatureStop"), Cmd_TemperatureStop); 
SerialCommandHandler.AddCommand(F("FlowStart"), Cmd_FlowStart); 
SerialCommandHandler.AddCommand(F("FlowStop"), Cmd_FlowStop); 
SerialCommandHandler.AddCommand(F("PhotoStart"), Cmd_PhotoStart); 
SerialCommandHandler.AddCommand(F("PhotoStop"), Cmd_PhotoStop); 
pinMode(SolenoidAPIN,OUTPUT); 
pinMode(SolenoidBPIN,OUTPUT); 
 
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
 
if ((ReactionRUN == true)&&(Capilliary == 1)) { 
 if (Solvent==1){
TotalTime=(((TotalVolume+InjectionVolume)/((0.018*Pressure)+0.0215))*60000); 
  TotalMillis=(StartMillis+TotalTime); 
  Eflowrate=((0.0168*Pressure)+0.02);
  if (millis() - StartMillis >= ((InjectionVolume/((0.018*Pressure)+0.0215))*60000)) { 
    digitalWrite(SolenoidAPIN,LOW); 
    digitalWrite(SolenoidBPIN,LOW); 
    ReactionRUN = false;  }
} 

 else if (Solvent==2){
TotalTime=(((TotalVolume+InjectionVolume)/((0.0407*Pressure)+0.085))*60000); 
  TotalMillis=(StartMillis+TotalTime); 
  Eflowrate=((0.0437*Pressure)+0.0913);
  if (millis() - StartMillis >= ((InjectionVolume/((0.0407*Pressure)+0.085))*60000)) { 
    digitalWrite(SolenoidAPIN,LOW); 
    digitalWrite(SolenoidBPIN,LOW); 
    ReactionRUN = false;  }
} 

  else if (Solvent==3){
TotalTime=(((TotalVolume+InjectionVolume)/((0.0291*Pressure)-0.005))*60000); 
  TotalMillis=(StartMillis+TotalTime); 
  Eflowrate=((0.0325*Pressure)-0.0065);
  if (millis() - StartMillis >= ((InjectionVolume/((0.0291*Pressure)-0.005))*60000)) { 
    digitalWrite(SolenoidAPIN,LOW); 
    digitalWrite(SolenoidBPIN,LOW); 
    ReactionRUN = false;  }
} 

  else if (Solvent==4){
TotalTime=(((TotalVolume+InjectionVolume)/((0.02156*Pressure)-0.04609))*60000); 
  TotalMillis=(StartMillis+TotalTime); 
  Eflowrate=((0.0229*Pressure)-0.049);
  if (millis() - StartMillis >= ((InjectionVolume/((0.02156*Pressure)-0.04609))*60000)) { 
    digitalWrite(SolenoidAPIN,LOW); 
    digitalWrite(SolenoidBPIN,LOW); 
    ReactionRUN = false;  }
} 
  else if (Solvent==5){
TotalTime=(((TotalVolume+InjectionVolume)/((0.0095*Pressure)-0.014))*60000); 
  TotalMillis=(StartMillis+TotalTime); 
  Eflowrate=((0.0112*Pressure)-0.0165);
  if (millis() - StartMillis >= ((InjectionVolume/((0.0095*Pressure)-0.014))*60000)) { 
    digitalWrite(SolenoidAPIN,LOW); 
    digitalWrite(SolenoidBPIN,LOW); 
    ReactionRUN = false;  }
}

  else if (Solvent==6){
TotalTime=(((TotalVolume+InjectionVolume)/((0.01424*Pressure)-0.02072))*60000); 
  TotalMillis=(StartMillis+TotalTime); 
  Eflowrate=((0.0178*Pressure)-0.0259);
  if (millis() - StartMillis >= ((InjectionVolume/((0.01424*Pressure)-0.02072))*60000)) { 
    digitalWrite(SolenoidAPIN,LOW); 
    digitalWrite(SolenoidBPIN,LOW); 
    ReactionRUN = false;   }
}
MyPanel.SetText(F("Progress"), ((millis()-StartMillis)/TotalTime)*100);
MyPanel.SetText(F("TotalTime"), (TotalTime/60000));
MyPanel.SetText(F("Eflowrate"), (Eflowrate));
} 
 
else if ((ReactionRUN == true)&&(Capilliary== 2)){ 

 if (Solvent==1){
TotalTime=(((TotalVolume+InjectionVolume)/((0.0375*Pressure)+0.0562))*60000); 
  TotalMillis=(StartMillis+TotalTime); 
  Eflowrate=((0.0357*Pressure)+0.0535);
  if (millis() - StartMillis >= ((InjectionVolume/((0.0375*Pressure)+0.0562))*60000)) { 
    digitalWrite(SolenoidAPIN,LOW); 
    digitalWrite(SolenoidBPIN,LOW); 
    ReactionRUN = false;  }
}
 else if (Solvent==2){
TotalTime=(((TotalVolume+InjectionVolume)/((0.0825*Pressure)+0.2004))*60000); 
  TotalMillis=(StartMillis+TotalTime); 
  Eflowrate=((0.0825*Pressure)+0.2004);
  if (millis() - StartMillis >= ((InjectionVolume/((0.0825*Pressure)+0.2004))*60000)) { 
    digitalWrite(SolenoidAPIN,LOW); 
    digitalWrite(SolenoidBPIN,LOW); 
    ReactionRUN = false;  }
}

else if (Solvent == 3){
TotalTime=(((TotalVolume+InjectionVolume)/((0.0513*Pressure)+0.0713))*60000); 
  TotalMillis=(StartMillis+TotalTime); 
  Eflowrate=((0.0513*Pressure)+0.0713);
  if (millis() - StartMillis >= ((InjectionVolume/((0.0513*Pressure)+0.0713))*60000)) { 
    digitalWrite(SolenoidAPIN,LOW); 
    digitalWrite(SolenoidBPIN,LOW); 
    ReactionRUN = false;  }
  } 

  else if (Solvent==4){
TotalTime=(((TotalVolume+InjectionVolume)/((0.03492*Pressure)+0.00324))*60000); 
  TotalMillis=(StartMillis+TotalTime); 
  Eflowrate=((0.0388*Pressure)+0.0036);
  if (millis() - StartMillis >= ((InjectionVolume/((0.03492*Pressure)+0.00324))*60000)) { 
    digitalWrite(SolenoidAPIN,LOW); 
    digitalWrite(SolenoidBPIN,LOW); 
    ReactionRUN = false;  }
} 

  else if (Solvent==5){  
TotalTime=(((TotalVolume+InjectionVolume)/((0.01696*Pressure)-0.00308))*60000); 
  TotalMillis=(StartMillis+TotalTime); 
  Eflowrate=((0.0193*Pressure)-0.0035);
  if (millis() - StartMillis >= ((InjectionVolume/((0.01696*Pressure)-0.00308))*60000)) { 
    digitalWrite(SolenoidAPIN,LOW); 
    digitalWrite(SolenoidBPIN,LOW); 
    ReactionRUN = false;  }
  }

  else if (Solvent==6){
TotalTime=(((TotalVolume+InjectionVolume)/((0.0244*Pressure)+0.1691))*60000); 
  TotalMillis=(StartMillis+TotalTime); 
  Eflowrate=((0.0244*Pressure)+0.1691);
  if (millis() - StartMillis >= ((InjectionVolume/((0.0244*Pressure)+0.1691))*60000)) { 
    digitalWrite(SolenoidAPIN,LOW); 
    digitalWrite(SolenoidBPIN,LOW); 
    ReactionRUN = false;  }
  }

MyPanel.SetText(F("Progress"), ((millis()-StartMillis)/TotalTime)*100);
MyPanel.SetText(F("TotalTime"), (TotalTime/60000));
MyPanel.SetText(F("Eflowrate"), (Eflowrate));
} 
 
else if ((ReactionRUN == true)&&(Capilliary == 3)) { 
 
 if (Solvent==1){
TotalTime=(((TotalVolume+InjectionVolume)/((0.1559*Pressure)+0.1585))*60000); 
  TotalMillis=(StartMillis+TotalTime); 
  Eflowrate=((0.01559*Pressure)+0.1585);
  if (millis() - StartMillis >= ((InjectionVolume/((0.1559*Pressure)+0.1585))*60000)) { 
    digitalWrite(SolenoidAPIN,LOW); 
    digitalWrite(SolenoidBPIN,LOW); 
    ReactionRUN = false;  }
}

 else if (Solvent==2){
TotalTime=(((TotalVolume+InjectionVolume)/((0.32*Pressure)+0.71))*60000); 
  TotalMillis=(StartMillis+TotalTime); 
  Eflowrate=((0.3364*Pressure)+0.7051);
  if (millis() - StartMillis >= ((InjectionVolume/((0.32*Pressure)+0.71))*60000)) { 
    digitalWrite(SolenoidAPIN,LOW); 
    digitalWrite(SolenoidBPIN,LOW); 
    ReactionRUN = false; }
}

else if (Solvent==3){
TotalTime=(((TotalVolume+InjectionVolume)/((0.238*Pressure)+0.395))*60000); 
  TotalMillis=(StartMillis+TotalTime); 
  Eflowrate=((0.2268*Pressure)+0.3761);
  if (millis() - StartMillis >= ((InjectionVolume/((0.238*Pressure)+0.395))*60000)) { 
    digitalWrite(SolenoidAPIN,LOW); 
    digitalWrite(SolenoidBPIN,LOW); 
    ReactionRUN = false; }
} 

  else if (Solvent==4){  
TotalTime=(((TotalVolume+InjectionVolume)/((0.1332*Pressure)+0.1992))*60000); 
  TotalMillis=(StartMillis+TotalTime); 
  Eflowrate=((0.1436*Pressure)+0.2123);
  if (millis() - StartMillis >= ((InjectionVolume/((0.1332*Pressure)+0.1992))*60000)) { 
    digitalWrite(SolenoidAPIN,LOW); 
    digitalWrite(SolenoidBPIN,LOW); 
    ReactionRUN = false; }
} 
  else if (Solvent==5){
TotalTime=(((TotalVolume+InjectionVolume)/((0.221*Pressure)+0.158))*60000); 
  TotalMillis=(StartMillis+TotalTime); 
  Eflowrate=((0.0784*Pressure)+0.085);
  if (millis() - StartMillis >= ((InjectionVolume/((0.221*Pressure)+0.158))*60000)) { 
    digitalWrite(SolenoidAPIN,LOW); 
    digitalWrite(SolenoidBPIN,LOW); 
    ReactionRUN = false;   }
} 

  else if (Solvent==6){
TotalTime=(((TotalVolume+InjectionVolume)/((0.411*Pressure)+0.319))*60000); 
  TotalMillis=(StartMillis+TotalTime); 
  Eflowrate=((0.3891*Pressure)+0.4028);
  if (millis() - StartMillis >= ((InjectionVolume/((0.411*Pressure)+0.319))*60000)) { 
    digitalWrite(SolenoidAPIN,LOW); 
    digitalWrite(SolenoidBPIN,LOW); 
    ReactionRUN = false;  }
} 
MyPanel.SetText(F("Progress"), ((millis()-StartMillis)/TotalTime)*100);
MyPanel.SetText(F("TotalTime"), (TotalTime/60000));
MyPanel.SetText(F("Eflowrate"), (Eflowrate));
} 
 
else if ((ReactionRUN == true)&&(Capilliary == 4)) { 

  if (Solvent==1){
TotalTime=(((TotalVolume+InjectionVolume)/((0.401*Pressure)+0.550))*60000); 
  TotalMillis=(StartMillis+TotalTime); 
  Eflowrate=((0.3904*Pressure)+0.5241);
  if (millis() - StartMillis >= ((InjectionVolume/((0.401*Pressure)+0.550))*60000)) { 
    digitalWrite(SolenoidAPIN,LOW); 
    digitalWrite(SolenoidBPIN,LOW); 
    ReactionRUN = false;  }
}
 else if (Solvent==2){
TotalTime=(((TotalVolume+InjectionVolume)/((0.9125*Pressure)+1.8792))*60000); 
  TotalMillis=(StartMillis+TotalTime); 
  Eflowrate=((0.9125*Pressure)+1.8792);
  if (millis() - StartMillis >= ((InjectionVolume/((0.9125*Pressure)+1.8792))*60000)) { 
    digitalWrite(SolenoidAPIN,LOW); 
    digitalWrite(SolenoidBPIN,LOW); 
    ReactionRUN = false;  }
}
 
 else if (Solvent==3){
TotalTime=(((TotalVolume+InjectionVolume)/((0.606*Pressure)+1.23))*60000); 
  TotalMillis=(StartMillis+TotalTime); 
  Eflowrate=((0.5513*Pressure)+1.1187);
  if (millis() - StartMillis >= ((InjectionVolume/((0.606*Pressure)+1.23))*60000)) { 
    digitalWrite(SolenoidAPIN,LOW); 
    digitalWrite(SolenoidBPIN,LOW); 
    ReactionRUN = false; }
} 

  else if (Solvent==4){  
TotalTime=(((TotalVolume+InjectionVolume)/((0.3093*Pressure)+0.5240))*60000); 
  TotalMillis=(StartMillis+TotalTime); 
  Eflowrate=((0.3362*Pressure)+0.5696);
  if (millis() - StartMillis >= ((InjectionVolume/((0.3093*Pressure)+0.5240))*60000)) { 
    digitalWrite(SolenoidAPIN,LOW); 
    digitalWrite(SolenoidBPIN,LOW); 
    ReactionRUN = false; }
} 

  else if (Solvent==5){  
TotalTime=(((TotalVolume+InjectionVolume)/((0.221*Pressure)+0.158))*60000); 
  TotalMillis=(StartMillis+TotalTime); 
  Eflowrate=((0.221*Pressure)+0.158);
  if (millis() - StartMillis >= ((InjectionVolume/((0.221*Pressure)+0.158))*60000)) { 
    digitalWrite(SolenoidAPIN,LOW); 
    digitalWrite(SolenoidBPIN,LOW); 
    ReactionRUN = false; }
} 

  else if (Solvent==6){
TotalTime=(((TotalVolume+InjectionVolume)/((1.395*Pressure)+1.145))*60000); 
  TotalMillis=(StartMillis+TotalTime);
  Eflowrate=((1.395*Pressure)+1.145); 
  if (millis() - StartMillis >= ((InjectionVolume/((1.395*Pressure)+1.145))*60000)) { 
    digitalWrite(SolenoidAPIN,LOW); 
    digitalWrite(SolenoidBPIN,LOW); 
    ReactionRUN = false; }
}
MyPanel.SetText(F("Progress"), ((millis()-StartMillis)/TotalTime)*100);
MyPanel.SetText(F("TotalTime"), (TotalTime/60000));
MyPanel.SetText(F("Eflowrate"), (Eflowrate));
} 

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

if ((millis()-StartMillis)/TotalTime < 1){ 
 MyPanel.SetText(F("Progress"), ((millis()-StartMillis)/TotalTime)*100);
  } 

else if ((millis()-StartMillis)/TotalTime == 1){ 
  MyPanel.SetText(F("Progress"), 100);
  } 

else if ((millis()-StartMillis)/TotalTime > 1){ 
  MyPanel.SetText(F("Progress"), 100);
  } MyPanel.SetProgress(F("ReactionProgressBar"), ((millis()-StartMillis)/TotalTime)*100);  
