/*
 * bootmagic.h — geteilter RAM-Marker zwischen HBW-Booter und HBWired-App
 * =====================================================================
 * Loest die WDRF-Mehrdeutigkeit. Ein Watchdog-Reset entsteht bei MEHREREN Anlaessen:
 *   - gewolltes Firmware-Update ('u' START_BOOTER)        <- HIER soll der Booter bleiben
 *   - Geraete-Reset ('!!' resetSystem) / App-Restart      <- App soll normal starten
 *   - haengende App, die vom eigenen Watchdog resettet wird
 * Der Booter sieht in allen Faellen nur WDRF=1 und kann sie per Reset-Flag nicht trennen.
 *
 * LOESUNG: NUR der 'u'-Handler der App legt kurz vor  wdt_enable(); while(1);  diesen
 * Marker in eine feste RAM-Zelle. Der Booter liest ihn ganz am Anfang von main(),
 * entwertet ihn sofort und bleibt nur dann im Update-Modus, wenn WDRF UND Marker gesetzt.
 *   'u'  (Update gewollt)          -> Marker gesetzt      -> Booter bleibt
 *   '!!' / Restart / WDT-Hang      -> Marker NICHT gesetzt -> Booter faellt in die App
 *   Power-off (Strom weg)          -> RAM verliert Inhalt  -> Marker weg -> App (korrekt!)
 * Genau diese Power-off-Semantik ist der Grund fuer RAM (.noinit-artig) statt EEPROM:
 * nach einem Stromausfall darf KEIN Update-Modus haengenbleiben.
 *
 * RAM-LAGE: die obersten 4 Byte (RAMEND-3 .. RAMEND).
 *   - Der BOOTER reserviert sie, indem sein Stack per Linker-Flag darunter beginnt:
 *         -Wl,--defsym=__stack=<RAMEND-4>     (siehe build.sh, je MCU)
 *     So ueberschreibt weder der C-Startup (main-Ruecksprungadresse) noch der Booter-Stack
 *     die Zelle, bevor sie gelesen ist.
 *   - Die APP braucht KEINEN Flag: sie schreibt den Marker direkt vor  wdt_enable(); while(1);
 *     und kehrt danach nie zurueck -- das dabei kurz ueberschriebene oberste Stack-Byte
 *     (crt/main-Kontext) wird nie wieder gebraucht.
 *
 * Wert + Adresse MUESSEN in Booter UND App identisch sein. Die App-Seite spiegelt diese
 * beiden Zeilen in HBWired.cpp (die Arduino-Lib inkludiert dieses Header nicht direkt).
 */
#ifndef BOOTMAGIC_H
#define BOOTMAGIC_H
#include <avr/io.h>   /* RAMEND, uint32_t */

#define BOOT_MAGIC_VAL   0xB007DA7AUL                     /* willkuerlicher Marker ("B00T-DATA") */
#define BOOT_MAGIC_CELL  (*(volatile uint32_t*)(RAMEND-3))

#endif /* BOOTMAGIC_H */
