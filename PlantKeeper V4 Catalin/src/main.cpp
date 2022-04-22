/*
   PLANT KEEPER V1.0
   

   Software is published for illustrative/learning purposes only.
   Feel free to reuse and modify for your own project.

   DISCLAIMER: ANY USE OF THIS CODE IS AT YOUR OWN RISK!

   Libraries needed:
   - BLYNK: http://help.blynk.cc/en/articles/512105-how-to-install-blynk-library-for-arduino-ide
   - Time: https://github.com/PaulStoffregen/Time
   - ADS1115_WE: Found in the builtin library manbager

   And you need to have installed support for esp8266 boards

   Enjoy!

 Versiuni
 V1
 -citeste DHT11 si le pune pe blink
 -calculeaza ok timpul de ON al releului de pompa, eventual poti schimba la functia waterdur timpii in ms
 Functioneaza deocamdata ptr PLANTE1 pe D7 activat, restul este dezactivat
 - activarea releului se face pe LOW la pin D7

 V2
 - incercam sa implementam wifi credentials de la pagina asta modelul2 : https://www.circuitschools.com/change-wifi-credentials-of-esp8266-without-uploading-code-again/
   inca nu e pus la punct
 - conectam si sensor de temp onewire dallas DS18d20
 ATENTIE: se pot conecta in serie mai multi sensori DS cu o singura resitenta de 4,7k pusa de la pinul DATA la +3.3V dar la mine
 nu a mers si l-am pus la +5V si este ok citindul corect

 V3
 - implementare tabul "Ventilatie"
  - slide vertical = timp de actionare driver blink(V20)
  - buton UP -> blink(V18) actioneaza GPIO14 si GPIO16 cu HIGH respectiv LOW
  - boton Down -> blynk(V19) idem ca mai sus dar LOW respectiv HIGH
*/



// Libraries
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h> // import Blynk library
#include <ADS1115_WE.h>   // first import library ADS1115_WB by Wolfgang Ebald or something
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h> // import DallasTemperature
#include <TimeLib.h>   // first import Time library by Paul Stoffregen
#include <WidgetRTC.h>
#include <SPI.h>
#include <DHT.h>  // DHT sensor library by Adafruit

// V2
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager

/* DS18B20 Temperature Sensor */
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 2    //Data wire plugged to pin D4=GPIO2 on wemosD1 (DS18B20 sensor) il incarcam pe blynk V17
#define DHTPIN 0     //Data wire plugged to pin GPIO0= D3(DHT22 sensor)
#define DHTTYPE DHT11     //DHT 22 Change this if you have a DHT11
DHT dht(DHTPIN, DHTTYPE); // creeaza obiectul dht
#define RELEU_1 14  //releul 1 pe pin GPIO14 =D5
#define RELEU_2 16 // releul 2 pe pin GPIO16 =D6
OneWire oneWire(ONE_WIRE_BUS);  // Setup a oneWire instance to communicate with any OneWire device
DallasTemperature sensors(&oneWire); // Pass oneWire reference to DallasTemperature library

// Networking
char auth[] =  "y1GQKtVrlwLcNTUmWrNydObwe-lM_BmI" ; //"ohN4AxXkxKODewRovQTchz6cMo9qy1H-";   
char ssid[] = "cyobyflo";
char pass[] = "11197104";

// Blynk
BlynkTimer timer;
BlynkTimer timer1;
BlynkTimer timer2;
BlynkTimer timer3;
//BlynkTimer timer3;
WidgetRTC rtc;

// System state
int plantSelect = 0;                            // plant 1 selected on default (index 0)
bool systemOn = 0;
bool systFlag = false;
long ontime;
long lastWater[4] = {0, 0, 0, 0};               // 0 means not set
bool pumpOn[4] = {false, false, false, false};
int amountTimedriver = 0;
int sliderValue0 = 0;
int sliderValue1 = 1;

// Settings (can be adjusted from app)
int modes[4] = {0, 0, 0, 0};
int amount[4] = {0, 0, 0, 0};
int interval[4] = {0, 0, 0, 0};
int thresh[4] = {0, 0, 0, 0};
int minInterval[4] = {0, 0, 0, 0};

// sensor/ADC
ADS1115_WE adc(0x48);
float tf = 1;                                 // trust factor for smoothing filter
float sensor[4] = {100, 100, 100, 100};         // Set highest start value to avoid unwanted triggers
float sensorDry[4] = {850, 850, 850, 850};  // Reading from when fully emerged in water
float sensorWet[4] = {400, 400, 400, 400};  // Reading from when in 'dry' air  INVERS

void water(int plant) ;
unsigned long waterdur (int plant) ;
bool plantCheck (int plant);
String getStatus (int plant) ;
float readHumid1();
void driverUP();
void driverDOWN();
void driverSTOP();

// BLYNK ///////////////////////////////////////

BLYNK_CONNECTED()
{
  // Synchronize unix-time on connection
  rtc.begin();
}


// IN-APP EVENT CALLS ///////////////////////
// for when the user presses any button in the app

//----------------------------------------------------------
// System on-off button event
BLYNK_WRITE(V7)
{
  // change system state
  systemOn = param.asInt();

  if (systemOn) {
    if (systFlag) {
      // system has just been turned on!
      systFlag = false;

      // set lastWater to now:
      ontime = now();
      Blynk.virtualWrite(V34, ontime, ontime, ontime, ontime);
      Blynk.syncVirtual(V34);
    }
    // system was turned on when connected -> do nothing
  } else {
    // system is off
    systFlag = true;
  }
}

//-------------------------------------------------------------
// Manual water button event
BLYNK_WRITE(V8)
{
  // if button was pressed and plant is eligible for water
  if (param.asInt() == 1 and now() - lastWater[plantSelect] > 5 and systemOn) {
    // execute water routine
    water(plantSelect);
  } else {
    // reset water button to unpressed state
    Blynk.virtualWrite(V8, 0);
  }
}

//---------------------------------------------------------------
// Reload button event
BLYNK_WRITE(V2) 
{
  // Reload requested -> update display values in app
  if (param.asInt() == 1) {
    Blynk.virtualWrite(V1, modes[plantSelect]);
    Blynk.virtualWrite(V3, amount[plantSelect]);
    Blynk.virtualWrite(V4, interval[plantSelect]);
    Blynk.virtualWrite(V5, thresh[plantSelect]);
    Blynk.virtualWrite(V9, minInterval[plantSelect]);
  }
}

//--------------------------------------------------------------
// Plant Select event
BLYNK_WRITE(V0)
{
  // Plant selected for edit -> store updated value
  plantSelect = param.asInt() - 1;

  // update displays
  Blynk.virtualWrite(V1, modes[plantSelect]);
  Blynk.virtualWrite(V3, amount[plantSelect]);
  Blynk.virtualWrite(V4, interval[plantSelect]);
  Blynk.virtualWrite(V5, thresh[plantSelect]);
  Blynk.virtualWrite(V9, minInterval[plantSelect]);
}

//------------------------------------------------------------
// Mode select event
BLYNK_WRITE(V1)
{
  // Store updated value depending on selected plant
  modes[plantSelect] = param.asInt();

  // save updated value on server
  Blynk.virtualWrite(V30, modes[0], modes[1], modes[2], modes[3]);
}

//------------------------------------------------------------
// Amount change event
BLYNK_WRITE(V3)
{
  // Store updated value depending on selected plant
  amount[plantSelect] = param.asInt();

  // save to server
  Blynk.virtualWrite(V31, amount[0], amount[1], amount[2], amount[3]);
}

//-------------------------------------------------------------
// Interval change event
BLYNK_WRITE(V4)
{
  // Store updated value depending on selected plant
  interval[plantSelect] = param.asInt();

  // save to server
  Blynk.virtualWrite(V32, interval[0], interval[1], interval[2], interval[3]);
}

//------------------------------------------------------------
// Threshold change event
BLYNK_WRITE(V5)
{
  // Store updated value depending on selected plant
  thresh[plantSelect] = param.asInt();

  // save to server
  Blynk.virtualWrite(V33, thresh[0], thresh[1], thresh[2], thresh[3]);
}

//------------------------------------------------------------
// Minimum interval change event
BLYNK_WRITE(V9)
{
  // Store updated value depending on selected plant
  minInterval[plantSelect] = param.asInt();

  // save to server
  Blynk.virtualWrite(V35, minInterval[0], minInterval[1], minInterval[2], minInterval[3]);
}



// GET CALLS //////////////////////////////
// used when 'sync' is called at startup 

BLYNK_WRITE(V30)
{
  // Get values from server:
  for (int i = 0; i < 4; i++) {
    modes[i] = param[i].asInt();
  }
}

BLYNK_WRITE(V31)
{
  // Get values from server:
  for (int i = 0; i < 4; i++) {
    amount[i] = param[i].asInt();
  }
}

BLYNK_WRITE(V32)
{
  // Get values from server:
  for (int i = 0; i < 4; i++) {
    interval[i] = param[i].asInt();
  }
}

BLYNK_WRITE(V33)
{
  // Get values from server:
  for (int i = 0; i < 4; i++) {
    thresh[i] = param[i].asInt();
  }
}

BLYNK_WRITE(V35)
{
  // Get values from server:
  for (int i = 0; i < 4; i++) {
    minInterval[i] = param[i].asInt();
  }
}

// Last Water - retreive values from server
BLYNK_WRITE(V34)
{
  // Get values from server:
  for (int i = 0; i < 4; i++) {
    lastWater[i] = param[i].asInt();
  }
}



//--------------------------------------------------
// STATUS DISPLAY FUNCTIONS /////////////////////
// when app request current plant status 

BLYNK_READ(V11)
{
  Blynk.virtualWrite(V11, getStatus(0));
  Serial.println(getStatus(0));
}
BLYNK_READ(V12)
{
  Blynk.virtualWrite(V12, getStatus(1));
}
BLYNK_READ(V13)
{
  Blynk.virtualWrite(V13, getStatus(2));
}
BLYNK_READ(V14)
{
  Blynk.virtualWrite(V14, getStatus(3));
}




//V3 -------------------======= V3 =======--------------------

// This function will be called every time Slider Widget
// in Blynk app writes values to the Virtual Pin 20
BLYNK_WRITE(V20)
{
  // Use of syncAll() will cause this function to be called
  // Parameter holds last slider value
  int sliderValue0 = param.asInt(); // assigning incoming value from pin v20 to a variable
    // You can also use:
    // String i = param.asStr();
    // double d = param.asDouble();

   Serial.print("V20 Slider value is: ");
  Serial.println(sliderValue0);
  sliderValue1 = sliderValue0;
}

BLYNK_WRITE(V18)
{
 // if button was pressed and plant is eligible for ventilation
  if (param.asInt() == 1) {    // conditie daca driverDOWN V19 este 0 
   long time_to_wait = sliderValue1*1000; // 3 seconds

    // execute driverUP routine
    driverUP();
    Blynk.virtualWrite(V18,1); //salvez pe blynk starea butoanelor
    Blynk.virtualWrite(V19,0);
    timer3.setTimeout(time_to_wait, [] () {
      driverSTOP();
      Blynk.virtualWrite(V18, 0); //dupa time_to_wait secunde fac driverSTOP si setez valoarea butonului pe  0
      });
  } else {
    // reset ventilation button to unpressed state
    driverSTOP();
    Blynk.virtualWrite(V18, 0);
    
  } 
}

BLYNK_WRITE(V19)
{
 // if button was pressed and plant is eligible for ventilation
  if (param.asInt() == 1) {    // conditie daca driverUP V19 este 0 s
  long time_to_wait = sliderValue1*1000; // 3 seconds

    // execute driverDOWN routine
    
    driverDOWN();
    Blynk.virtualWrite(V18,0);
    Blynk.virtualWrite(V19,1);
    timer3.setTimeout(time_to_wait, [] () {
      driverSTOP();
      Blynk.virtualWrite(V19, 0);
      });
  } else {
    // reset ventilation button to unpressed state
    driverSTOP();
    Blynk.virtualWrite(V19, 0);
  } 
}
//-------------------- 3_control.ino

void control () 
{
  // check if system is on
  if (systemOn) {
    
    // check if it's time to water
    for (int i = 0; i < 4; i++) {
      if (plantCheck(i)) {
        // yes -> execute water routine
        water(i);
      }
    }
  }
}


bool plantCheck (int plant) 
{
  // find elapsed time since last water 
  long elapsedTime = now() - lastWater[plant];

  // check if in auto mode
  if (modes[plant] == 2 and elapsedTime > minInterval[plant] * 60 * 60 and sensor[plant] < thresh[plant]) {
    // minimun interval exceed and moisturelevels too low -> time to water!
    return true;
  }

  // check if in timer mode
  else if (modes[plant] == 3 and elapsedTime > interval[plant] * 60 * 60 * 24) {
    // time interval has been exceeded -> time to water:
    return true;
  }

  // no hit, no water
  return false;
}


void water(int plant) 
{
  Serial.print("Udăm plantele "); Serial.println(plant + 1);

  // set flag (used for status update)
  pumpOn[plant] = true;

  // push-update status in app and then turn pump on, and
  switch (plant) {
    case 0:
      Blynk.virtualWrite(V11, getStatus(plant)); // Plant1
      digitalWrite(D7, HIGH);
      break;
    case 1:
      Blynk.virtualWrite(V12, getStatus(plant)); // Plant2
      //digitalWrite(D5, LOW);
      break;
    case 2:
      Blynk.virtualWrite(V13, getStatus(plant)); // Plant3
      //digitalWrite(D4, LOW);
      break;
    case 3:
      Blynk.virtualWrite(V14, getStatus(plant)); // Plant4
      //digitalWrite(D6, LOW);
      break;
  }

  // delay loop
  unsigned long startTime = millis();
  while ((millis() - startTime) < waterdur(plant)) {
    // keep everything running in the meantime (except the control loop)
    Blynk.run();
    timer1.run();
  }

  // remove flag
  pumpOn[plant] = false;

  // turn pump off and push new status
  switch (plant) {
    case 0:
      digitalWrite(D7, LOW);  
      Blynk.virtualWrite(V11, getStatus(plant));
      break;
    case 1:
      //digitalWrite(D5, HIGH);  
      Blynk.virtualWrite(V12, getStatus(plant));
      break;
    case 2:
      //digitalWrite(D4, HIGH);  
      Blynk.virtualWrite(V13, getStatus(plant));
      break;
    case 3:
      //digitalWrite(D6, HIGH);  
      Blynk.virtualWrite(V14, getStatus(plant));
      break;
  }

  // Reset water button (if used)
  Blynk.virtualWrite(V8, 0);

  // update lastwater to server
  lastWater[plant] = now();
  Blynk.virtualWrite(V34, lastWater[0], lastWater[1], lastWater[2], lastWater[3]);
}


unsigned long waterdur (int plant) 
{
  // convert ml to ms for controlling pump on-time, different values depending on the chosen pump

  switch (plant) {
    case 0:
      return amount[plant] * 57 * 100 + 3500; // set experimentally  tON = VmL * G(ms/mL) + Kms// am pus 100 si am la 1L 4sec si la 10L 40sec
    case 1:
      return amount[plant] * 50* 100 + 3200; // set experimentally
    case 2:
      return amount[plant] * 59 *100 + 3500; // set experimentally
    case 3:
      return amount[plant] * 47 *100 + 3400; // set experimentallyl
  } 
  return 0;
}



//--------------------- 4_sensors.ino

void readSensors() 
{
  float reading[4];
  int tempDS18;
  // read raw values
  //reading[0] = readChannel(ADS1115_COMP_0_GND); //cand folosim ADS1115
    reading[0] = readHumid1();
    Serial.print("Analog read[0]->");
    Serial.print(reading[0]);
   // Serial.print(" = Map ");
    reading[0] = map(reading[0], sensorDry[0], sensorWet[0], 0, 100);
    //Serial.println(reading[0]);
    sensor[0] = tf * reading[0] + (1 - tf) * sensor[0];
    sensor[0] = constrain(sensor[0], 0, 100);
    Serial.print(" Umiditate Sol = ");
    Serial.print(sensor[0]);
    Serial.println("%");
    
    
  //reading[1] = readChannel(ADS1115_COMP_1_GND); //cand folosim ADS1115
    reading[1] = 200; // setat la valoare aleatoare 500
    
  //reading[2] = readChannel(ADS1115_COMP_2_GND); //cand folosim ADS1115
    reading[2] = 500;
    
  //reading[3] = readChannel(ADS1115_COMP_3_GND); //cand folosim ADS1115
    reading[3] = 800;

  // convert to percentage, filter and constrain (if we got a rouge reading) pentru restul de 3 senzori DEBUG
   Serial.println(".... Mai jos 3 valori de la ceilalti viitori sensori...");
  for (int i = 1; i<4; i++) {
    reading[i] = map(reading[i], sensorDry[i], sensorWet[i], 0, 100);
    sensor[i] = tf * reading[i] + (1 - tf) * sensor[i];
    sensor[i] = constrain(sensor[i], 0, 100);
    Serial.println(sensor[i]);
  }

//DS18 sensor
Serial.print("Requesting DS18 temperatures...");
  sensors.requestTemperatures(); // Send the command to get temperatures
  Serial.println("DONE");

  Serial.print("Temperatura ambient data de DS18 index0 este ");
  Serial.print(sensors.getTempCByIndex(0));
  tempDS18 = sensors.getTempCByIndex(0);
  Serial.println("°C"); // press ALT and 0176 numeric keypad to generate celsius sign
  Serial.println("..................................");
  
  // Write to server
  Blynk.virtualWrite(V21, sensor[0]); 
  Blynk.virtualWrite(V22, sensor[1]); 
  Blynk.virtualWrite(V23, sensor[2]); 
  Blynk.virtualWrite(V24, sensor[3]); 
  Blynk.virtualWrite(V17, tempDS18);
}


float readChannel(ADS1115_MUX channel) 
{
  float voltage = 0.0;
  adc.setCompareChannels(channel);
  adc.startSingleMeasurement();
  while (adc.isBusy()) {
    Blynk.run(); // nope!
  }
  voltage = adc.getResult_mV(); 
  return voltage;
}
//------------------------- read MOISTURE --direct from analog A0
float readHumid1()
{
 float soilMoistureValue = 0.0;
 soilMoistureValue = analogRead(A0); //Mention where the analog pin is connected on NodeMCU 
 return soilMoistureValue;
}

//------------ 5_messages.ino

String getStatus (int plant) 
{
  // string variables
  String mode = "";
  String status = "";

  if (pumpOn[plant] == true) {
    // pump is on
    return "Irigăm ...";
    
  } else if (systemOn) {
    // determine mode
    switch (modes[plant]) {
      case 0:
        // not set up yet
        return "Nu e setat încă!";
        break;
      case 1:
        mode = "Manual";
        break;
      case 2:
        mode = "Auto";
        break;
      case 3:
        mode = "Timer";
        break;
    }

    // Determine time of last water
    if (lastWater[plant] == ontime) {
      status = "Neirigat.";
    } else {
      // calculate difference
      long diff = now() - lastWater[plant];

      // onvert between minutes, hours, days and too much
      if (diff < 60) {
        status = "chiar acum.";

      } else if (diff < 2 * 60) {
        status = "acum 1 minut.";
        
      } else if (diff < 60 * 60) {
        status = String(diff / 60) + " minute ago.";
        
      } else if (diff < 60 * 60 * 2) {
        status = "1 hour ago";

      } else if (diff < 60 * 60 * 24) {
        status = String(diff / (60 * 60)) + " hours ago.";

      } else if (diff < 60 * 60 * 24 * 2) {
        status = "1 day ago";

      } else if (diff < 60 * 60 * 24 * 31) {
        status = String(diff / (60 * 60 * 24)) + " days ago.";

      } else {
        status = "+1 month ago.";
      }
    }

    // return concatenated results
    return mode + ",  " + status;

  } else {
    // system is off
    return "Off";
  } 
}

/*/----------------void read DHT11
void sendDHTSensor()
{
  float h = dht.readHumidity();
  float t = dht.readTemperature(); // or dht.readTemperature(true) for Fahrenheit

  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  Blynk.virtualWrite(V15, t);
  Blynk.virtualWrite(V16, h);
  Serial.print("DHT temp: ");
  Serial.print(t);
  Serial.print(" / Umid: ");
  Serial.println(h);
}
*/
//----------------------------------------------
void sendDHTSensor()
{
  //static unsigned long twas=0,tnow=0;
 sensors.requestTemperatures();
 float floatTempC = sensors.getTempCByIndex(0); //Stores temperature. Change to getTempFByIndex(0) for F.
 char t_buffer[15];
 dtostrf(floatTempC, 8, 9, t_buffer);


 // Check if any reads failed and exit early (to try again).

 float h = dht.readHumidity();
 float t = dht.readTemperature();
 
  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  Serial.print("Temp DHT: ");
 Serial.print(t,2); 
 Serial.print(" / Umid DHT: ");  Serial.println(h,2);
  
 //Blynk.virtualWrite(5, t_buffer); //DS18W20 temperature virtual pin V5
 Blynk.virtualWrite(15, t);     //DHT21 temperature virtual pin V15
 Blynk.virtualWrite(16, h);     //DHT21 humidity virtual pinV16


 /*/Init OLED screen
  u8x8.begin();
  u8x8.setPowerSave(0);
  //u8x8.setFont(u8x8_font_inr46_4x8_n); 
  u8x8.setFont(u8x8_font_inb46_4x8_n); 
  //u8x8.setFont(u8x8_font_chroma48medium8_r);
   u8x8.clear();
  u8x8.setFlipMode(1);
  u8x8.setInverseFont(0);
  char resultTemp[10];
          dtostrf(t, 4, 1, resultTemp); //4 is mininum width, 2 is precision; float value is copied onto buff
      
          char resultHum[10];
          dtostrf(h, 4, 1, resultHum);//4 is mininum width, 2 is precision; float value is copied onto buff
  //drawCentered(0, resultHum);
  u8x8.print(resultHum);      // Arduino Print function
  */
}

//-- buton UP ventilatie ---------------
void driverUP()
{
    digitalWrite(RELEU_1, HIGH);
    digitalWrite(RELEU_2, LOW);

}

//-- buton DOWN ventilatie --------------
void driverDOWN()
{
    digitalWrite(RELEU_1,LOW);
    digitalWrite(RELEU_2,HIGH);

}

//-- buton STOP ventilatie ---------------
void driverSTOP()
{
    digitalWrite(RELEU_1, HIGH);
    digitalWrite(RELEU_2, HIGH);

}

//====================================================================
//====================================================================
//---------------------- 0_setup.ino
void setup()
{

// Serial
  Serial.begin(9600);
  
//  versiunea V3
WiFiManager wifiManager;
WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
                     // it is a good practice to make sure your code sets wifi mode how you want it.
                     
     //WiFiManager, Local intialization. Once its business is done, there is no need to keep it around
 WiFiManager wm; 

    // reset settings - wipe stored credentials for testing
    // these are stored by the esp library
  //  wm.resetSettings();

 bool res;
 res = wm.autoConnect("RaFix_AP"); // anonymous ap

 if(!res) {
        Serial.println("Failed to connect");
        // ESP.restart();
    } 
    else {
        //if you get here you have connected to the WiFi    
        Serial.println("connected...yeey :)");
        //Serial.println("connected...yeey :)");
    }
  Serial.println("local ip");
  Serial.println(WiFi.localIP());                  

  

  // Blynk 
  Blynk.begin(auth, ssid, pass);
 Serial.println("Connecting to ");
  Serial.println(ssid);
  // fetch stored data from server
  Blynk.syncVirtual(V0);
  Blynk.syncVirtual(V7);
  Blynk.syncVirtual(V30);
  Blynk.syncVirtual(V31);
  Blynk.syncVirtual(V32);
  Blynk.syncVirtual(V33);
  Blynk.syncVirtual(V34);
  Blynk.syncVirtual(V35);
  Blynk.syncVirtual(V36); //ammount driver time

  // Sensors
 /* Wire.begin(D1, D2);  // SDA, SCL
  if (!adc.init()) {
    Serial.println("ADS1115 not connected!");
  }
  adc.setVoltageRange_mV(ADS1115_RANGE_6144);
  */
  Serial.println("Initialize DHT22...");
  dht.begin();
  sensors.setResolution(10);   //More on resolution: http://www.homautomation.org/2015/11/17/ds18b20-how-to-change-resolution-9101112-bits/
  timer2.setInterval(5000L, sendDHTSensor); //for DHT sensor
  
  // timers
  timer.setInterval(30000L, control);     // for control loop, run every 30 secs
  timer1.setInterval(5000L, readSensors); // for sensor read loop, run every 5 sec
  //sendDHTSensor();
  

  // RTC
  setSyncInterval(10 * 60);               // Sync interval in seconds (10 minutes)
  
  // pin assignments for relays
  pinMode(D7, OUTPUT);
    //pinMode(D4, OUTPUT);
    //pinMode(D5, OUTPUT);
    //pinMode(D6, OUTPUT);

  // set pumps to OFF (active-low)
  digitalWrite(D7, LOW);
    //digitalWrite(D4, HIGH);
    //digitalWrite(D5, HIGH);
    //digitalWrite(D6, HIGH);

    // pin assignments for relay
    pinMode(RELEU_1, OUTPUT);
    pinMode(RELEU_2, OUTPUT);

   // set airflow driver to OFF --- vezi descrierea pe poza-- D14 D16 egale ca iesire => driver OFF
    digitalWrite(RELEU_1, HIGH);
    digitalWrite(RELEU_2, HIGH);

  // Set a reasonable start value for sensors (a little above the triggering threshold)
  for (int i = 0; i < 4; i++) {
    sensor[i] = thresh[i] + 10;
  }

  Serial.println("-----------------Setup Complete----------------");
}


//------------------------------------ 1_loop.ino
void loop()
{
  // main blynk loop
  Blynk.run();

  // timers
  timer.run();
  timer1.run();
  timer2.run();
  timer3.run();

 /* Serial.print("Requesting temperatures...");
  sensors.requestTemperatures(); // Send the command to get temperatures
  Serial.println("DONE");

  Serial.print("Temperature for the device 1 (index 0) is: ");
  Serial.println(sensors.getTempCByIndex(0));
  Serial.print("Temperature for the device 2 (index 1) is: ");
  Serial.println(sensors.getTempCByIndex(1));
  Serial.print("Temperature for the device 3 (index 2) is: ");
  Serial.println(sensors.getTempCByIndex(2));  
  */
 
}
