# HBW-Booter — Over-the-Bus-Firmware-Update für HMW/HBWired-Geräte

Eigener Bootloader für **ATmega328P/328PB, ATmega32A, ATmega644P/644PA und ATmega1284P**, mit dem sich
HomeMatic-Wired-Eigenbau-Geräte (HBWired) **über den RS485-Bus flashen** lassen — ohne ISP, ohne
Ausbau. Der Booter wird **einmalig** per ISP eingespielt; danach läuft jedes weitere Firmware-Update
über die Leitung. Alle Details: [maxx3105:HBW-Booter](https://github.com/maxx3105/HBW-Booter)


## Hier zusätzlich:
* `build.bat` Zum kompilieren unter Windows. Nutzt _arduino\tools\avr-gcc_
* documentation\arduino_nano, details zu fuses
* Leicht angepasste `hbw_booter.c`, mit Pinout passend zu den Beispiel Devices. In der USE_HARDWARE_SERIAL Variante (der Bus muss mit **UART0** verbunden sein!)
