# Plant-Keeper-V4
Varianta V4 pentru Plant Keeper (Catalin)

ONE_WIRE_BUS 2    //Data wire plugged to pin D4=GPIO2 on wemosD1 (DS18B20 sensor) il incarcam pe blynk V17
DHTPIN 0     //Data wire plugged to pin GPIO0= D3(DHT22 sensor)
RELEU_1 14  //releul 1 pe pin GPIO14 =D5
RELEU_2 16 // releul 2 pe pin GPIO16 =D6
D7  // pompa apa irigare


Porturi virtuale folosite

Plant Select event           V0
V1, modes[plantSelect]       V1 V30
Reload button event          V2
V3, amount[plantSelect]      V3 V31
V4, interval[plantSelect     V4 V32
V5, thresh[plantSelect       V5 V33
--------liber------          V6
System on-off button event   V7
Manual water button event    V8
V9, minInterval[plantSelect  V9 V35
-------liber-------          V10
STATUS DISPLAY FUNCTIONS     V11, V12,V13,V14
-----???-------              V15
-----???-------              V16
tempDS18                     V17
UP button                    V18
DOWN button                  V19
Slider timer                 V20
Sensori viitor               V21
Sensori viitor               V22
Sensori viitor               V23
Sensori viitor               V24
------Liber---------         V25-V29

Mode select event            V30
                             V31
			     V32
			     V33

set lastWater                V34
,,,,,,,,,,,,,,,,             V35