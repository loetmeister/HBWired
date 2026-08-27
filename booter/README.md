# HBW-Booter — Over-the-Bus-Firmware-Update für HMW/HBWired-Geräte

Eigener Bootloader für **ATmega328P/328PB, ATmega32A, ATmega644P/644PA und ATmega1284P**, mit dem sich
HomeMatic-Wired-Eigenbau-Geräte (HBWired) **über den RS485-Bus flashen** lassen — ohne ISP, ohne
Ausbau. Der Booter wird **einmalig** per ISP eingespielt; danach läuft jedes weitere Firmware-Update
über die Leitung. Alle Details: [maxx3105:HBW-Booter](https://github.com/maxx3105/HBW-Booter)


## Hier zusätzlich:
* `build.bat` Zum kompilieren unter Windows. Nutzt _arduino\tools\avr-gcc_
* `documentation\arduino_nano`, details zu fuses
* Leicht angepasste `hbw_booter.c`, mit Pinout passend zu den Beispiel Devices in der USE_HARDWARE_SERIAL Variante (der Bus muss mit **UART0** verbunden sein!).
  * USE_BUTTON 1 = Konfig Taster aktiv (Mit gedrücktem Taster beim Einschalten / Reset lässt sich der start des Booters erzwingen. Dann hat man ca. 25 Sekunden Zeit ein neues Firmware-Update zu starten)


### HBW Firmware-Update über den Bus:
**⚠ Die hex Datei muss zum Gerät passen. Typ, MCU, o.ä. wird nicht validiert!**
* `HM485_fwUpdate.pm` - Integration mit FHEM
* `flash_tool.py` - nutzt eine lokale serielle Schnittstelle (bzw. USB->Seriell Wandler)
 * Bsp. Aufruf um das Gerät mit Adresse 0x42000077 zu aktualisieren: `python.exe flash_tool.py COM3 "..\projekte\HBWired\HBW-LC-BL-4\HBW-LC-BL-4.ino.UART_v0.5.hex 0x42000077`
* Weitere Details: [github.com/maxx3105/HBW-Booter: Ab jetzt: über den Bus flashen — kein ISP mehr](https://github.com/maxx3105/HBW-Booter#2-ab-jetzt-%C3%BCber-den-bus-flashen--kein-isp-mehr)
