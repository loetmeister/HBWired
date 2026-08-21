@echo off
REM !!! See source: https://github.com/maxx3105/HBW-Booter  !!!
rem HBW-Booter Build — avr-gcc, 2048 Words / 4 KB Boot-Section (BOOTSZ=00).
rem Boot-Section-Start je MCU (= FLASHEND+1-0x1000): 32A/328P/328PB @0x7000 (32 KB), 644P @0xF000
rem (64 KB), 1284P @0x1F000 (128 KB). Muss zum BOOT_START im Code passen.
rem Fuses (einmalig per ISP): BOOTRST aktiv + BOOTSZ = 2048 Words.
rem ATmega328PB: eigenes avr-gcc-Target + eigene Signatur (0x1E9516), fuer den Booter aber
rem 328P-kompatibel (USART0/MCUSR/TIFR1 gleiche Adressen, FLASHEND 0x7FFF -> Boot @0x7000).
REM Zusaetzliche ATmega328PB variante mit 12MHz clock speed!

set AVR_GCC_BIN="C:\Users\user\AppData\Local\Arduino15\packages\arduino\tools\avr-gcc\7.3.0-atmel3.6.1-arduino7\bin"
set "FCPU=16000000UL"

setlocal enabledelayedexpansion

for %%M in (atmega32 atmega328p atmega328pb atmega644p atmega1284p atmega328pb_12MHz) do (
  set "MCU=%%M"
  set "SUFFIX="

  REM 328P/328PB: 32 KB Flash, 2 KB SRAM
  set "BOOT=0x7000"
  set "STACKTOP=0x08FB"

  REM 128 KB Flash, 16 KB SRAM
  if /I "!MCU!"=="atmega1284p" (set "BOOT=0x1F000" & set "STACKTOP=0x40FB")
  REM 64 KB Flash, 4 KB SRAM
  if /I "!MCU!"=="atmega644p"  (set "BOOT=0xF000"  & set "STACKTOP=0x10FB")
  REM 32 KB Flash, 2 KB SRAM (RAMEND 0x085F!)
  if /I "!MCU!"=="atmega32"    (set "BOOT=0x7000"  & set "STACKTOP=0x085B")
  REM atmega328pb 12MHz clock speed!
  if /I "!MCU!"=="atmega328pb_12MHz"    (set "FCPU=12000000UL" & set "MCU=atmega328pb" & set "SUFFIX=@12MHz")
  
  "%AVR_GCC_BIN%/avr-gcc.exe" -mmcu=!MCU! -DF_CPU=!FCPU! -Os -Wall -ffreestanding -Wl,--section-start=.text=!BOOT! -Wl,--defsym=__stack=!STACKTOP! -o hbw_booter_!MCU!!SUFFIX!.elf hbw_booter.c || exit /b 1
  "%AVR_GCC_BIN%/avr-objcopy.exe" -O ihex -R .eeprom hbw_booter_!MCU!!SUFFIX!.elf hbw_booter_!MCU!!SUFFIX!.hex
  echo "=== !MCU! (Boot @!BOOT!, FCPU: !FCPU!) ==="
  "%AVR_GCC_BIN%/avr-size.exe" hbw_booter_!MCU!!SUFFIX!.elf
)
echo ----------------------------------------------------------------------------------
echo done!
pause
