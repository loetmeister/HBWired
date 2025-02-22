Homematic Wired Homebrew Wetterstation
======================================

Dieses Modul stellt die Messwerte einer Bresser 7 in 1 [oder 5 in 1 - ToDo] Wetterstation als Homematic Wired Ger�t zur Verf�gung.
Die Code-Basis ist ein SIGNALDuino: https://github.com/Ralf9/SIGNALDuino/tree/dev-r335_cc1101 mit cc1101 868 MHz Modul.
Es kann parallel am RS-485 Bus und USB betrieben werden. Die Funktion des SIGNALDuino (advanced) ist nicht eingeschr�nkt.
Entwickelt auf/f�r einen Raspberry Pi Pico. Kompiliert mit dem Arduino Boards von earlephilhower: https://github.com/earlephilhower/arduino-pico (4.3.1)


Damit FHEM das Homebrew-Device richtig erkennt, muss die Datei hbw_wds_c7.xml in den Ordner \FHEM\lib\HM485\Devices\xml kopiert werden (Das Device gibt sich als HW-Typ 0x88 aus).

Config der Kan�le kann �ber das FHEM-Webfrontend vorgenommen werden:
# HBWSIGNALDuino
- Bisher keine Konfig (TODO: frequency, rfmode, etc.)

# Weather
* SENSOR_ID (wird automatisch gesetzt)
* STORM_THRESHOLD_LEVEL
* STORM_TRIGGER_COUNT
* SEND_DELTA_TEMP
* RX_TIMEOUT
* SEND_MIN_INTERVAL
* SEND_MAX_INTERVAL


Device Neustart mit set RAW "!!" (hexstring 2121) m�glich.


Pinbelegung:
0 - Rx Serial -> USB
1 - Tx Serial -> USB
7 - RS485 Enable
8, 9 (UART1) - RS485 bus
6 LED - HBW
20 - gdo0Pin TX out PIN_SEND              
21 - gdo2 PIN_RECEIVE           
22 - Bedientaster (Reset)
25 - PIN_LED (LED_BUILTIN) - SIGNALDuino
I2C EEPROM
SDA = PIN_WIRE0_SDA;
SCL = PIN_WIRE0_SCL;
cc1011 module:
SS = PIN_SPI0_SS;
MOSI = PIN_SPI0_MOSI;
MISO = PIN_SPI0_MISO;
SCK = PIN_SPI0_SCK;


FHEM:
Eigener Status in FHEMWEB und zus�tzliches reading "rain_total" um auch nach Batteriewechsel die Gesamtregenmenge zu erhalten.
attr Wetterstation stateFormat Timeout timeout | Batt battery
attr Wetterstation userReadings rain_total monotonic {ReadingsNum($name,"rain_mm",0);;;;;;;;}
