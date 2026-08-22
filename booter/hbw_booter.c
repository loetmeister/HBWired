/*
 * HBW-Booter — Over-the-Bus-Bootloader für HMW/HBWired-Geräte
 * ============================================================
 * Ziel-MCUs: ATmega32A (Produktivgeräte) UND ATmega328P (Entwicklung, Nano/Uno).
 * Meilenstein 1: Reset-Erkennung + Wire-Protokoll + Announce — OHNE echtes Flashen
 *                (WRITE_FLASH quittiert nur, wie DRY_RUN). Flashen = Meilenstein 2.
 *
 * Wire-Logik 1:1 aus HBW-Booter-Sim: Startbyte 0xFD, 0xFC-Escaping, CRC16 Poly 0x1002
 * (MSB-first), Bus 19200 8E1. Auf direkte UART-Register portiert (kein Arduino-Framework).
 *
 * EINSTIEG (der Kern, den der Sim NICHT leisten konnte):
 *   - Power-on / Brown-out / externer Reset  -> App starten (jmp 0)
 *   - Watchdog-Reset (von der App per 'u' ausgeloest) -> im Booter BLEIBEN
 *   Das ist die Reset-QUELLEN-Erkennung (kein Timing-Fenster).
 *
 * Build: avr-gcc, Boot-Section @0x7000 (2048 Words / 4 KB). Siehe build.sh.
 * Fuses (einmalig per ISP): BOOTRST aktiv + BOOTSZ = 2048 Words.
 * (App reicht damit bis 0x6FFF; Versionsfeld/fwmap-Marker entsprechend nach 0x6FF0.)
 */

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/boot.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <string.h>
#include "bootmagic.h"    /* geteilter RAM-Marker: 'u'-Update von '!!'-Reset unterscheiden */

/* ======================= KONFIG (hier anpassen) ======================= */
#ifndef F_CPU
#define F_CPU        16000000UL      /* HMW-Geraete: ext. 16-MHz-Quarz */
#endif
#define BUS_BAUD     19200UL

#define DEVICE_TYPE  0x00            /* NEUTRAL -- der Booter ist geraeteunabhaengig (ein Booter je MCU
                                        gilt fuer ALLE HBW-Geraete des Typs). Wird nur gemeldet, wenn ein
                                        Geraet OHNE laufende App abgefragt wird; die App meldet sonst ihren
                                        eigenen Typ. Fuers Flashen (Adresse kommt aus dem EEPROM) egal. */
#define HW_VERSION   0x00
#define FW_VERSION   0x0004          /* Booter-eigene Version, gemeldet bei 'v'. 0x0004: optionaler Konfig Taster (USE_BUTTON = 1) */
#define FALLBACK_ADDR 0x42FFFFFFUL   /* falls EEPROM-Adresse leer (0xFFFFFFFF) */

/* Inaktivitaets-Timeout: nach IDLE_TIMEOUT_OVF Timer1-Ueberlaeufen (je ~4,19 s
   @16 MHz, Prescaler 1024) ohne Bus-Aktivitaet faellt der Booter in eine INTAKTE
   App zurueck (Reset-Vektor gesetzt). So bleibt ein Geraet nach einem abgebrochenen
   Update nicht ewig im Booter haengen. 6 ~ 25 s -- lang genug, dass kein laufendes
   Update faelschlich abbricht (waehrend des Flashs ist die App ohnehin ungueltig). */
#define IDLE_TIMEOUT_OVF 6

/* RS485 Sende-Enable (DE) -- MUSS zum Pin der Platine passen!
 * Stimmt der Pin NICHT, ist der Fehler heimtueckisch: der Booter EMPFAENGT normal (RX lauscht
 * passiv), kann aber NIE senden, weil der Transceiver auf Empfang stehen bleibt. Das Geraet ist
 * dann am Bus komplett unsichtbar (Discovery findet 0 Geraete), waehrend die App -- die den
 * richtigen Pin benutzt -- voellig normal antwortet. Symptom beim Update: die App quittiert das
 * 'u' noch, danach kommt auf p/w nie ein ACK ("kein ACK @<erste Blockadresse>").
 * Abgleich mit dem Sketch: dessen RS485_TXEN muss denselben Portpin meinen.
 *   328P/328PB : Arduino-Pin 2  = PD2   (manche Platinen Pin 3 = PD3 -> hier anpassen!)
 *   32A/644P/1284P: Arduino-Pin 12 = PD4 (MightyCore-Standard-Pinout)
 * USE_DE 0 = Auto-Direction-Modul ohne DE-Pin. */
#define DE_DDR   DDRD
#define DE_PORT  PORTD
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328PB__) || defined(__AVR_ATmega328__)
  #define DE_BIT 2                   /* PD2 */
#else
  #define DE_BIT 4                   /* PD4 -- 32A / 644P / 1284P (MightyCore) */
#endif
#define USE_DE   1

/* Status-LED: zeigt an, dass der BOOTER laeuft (die App blinkt so nie) -- damit sieht man von
 * aussen sofort, ob ein Geraet im Update-Modus haengt, statt es am Bus suchen zu muessen.
 *   wartend        : kurzer Blitz ~1x/s   ("Booter aktiv, kein Update im Gang")
 *   Update laeuft  : schnelles Blinken    (ab dem ersten 'w'/'r' -- Daten fliessen)
 * Muss wie DE zum Pin der Platine passen (Abgleich mit LED im Sketch-Config):
 *   328P/328PB     : PB5 = Arduino D13 (LED_BUILTIN)
 *   32A/644P/1284P : PD2 = Arduino-Pin 10 (MightyCore-Standard)
 * Aktiv HIGH wie in HBWired (digitalWrite(ledPin,HIGH) = an); bei Low-Side-LED auf 1 setzen.
 * USE_LED 0 = LED-Ansteuerung komplett aus (z.B. wenn der Pin anders belegt ist). */
#define USE_LED         1
#define LED_ACTIVE_LOW  0
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328PB__) || defined(__AVR_ATmega328__)
  #define LED_DDR  DDRB
  #define LED_PORT PORTB
  #define LED_BIT  5                 /* PB5 */
#else
  #define LED_DDR  DDRD
  #define LED_PORT PORTD
  #define LED_BIT  2                 /* PD2 */
#endif

/* Konfig Taster: Taster der im Sketch fuer Factory-Reset etc. genutzt wird. Beim Reset
 * oder Poweron gedrueckt halten, um den start des booters zu erzwingen.
 * Muss zum Pin der Platine passen (Abgleich mit BUTTON im Sketch-Config):
 *   328P      : ADC6 = Arduino (analogRead A6)
 *   328PB     : PE2 = Arduino A6
 *   32A/644P/1284P : PB0 = Arduino-Pin 8 (MightyCore-Standard ???) - TODO: ueberpruefen & testen
 * Aktiv LOW; USE_BUTTON 0 = Konfig Taster im Booter nicht genutzt. 1 = nutzbar */
#define USE_BUTTON         1
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__)
  #define BUTTON_DDR DDRC               /* nicht benutzt bei ADC */
  #define BUTTON_PORT PINC               /* nicht benutzt bei ADC */
  #define BUTTON_PIN PORTC               /* nicht benutzt bei ADC */
  #define BUTTON_ADC_ONLY
  #define BUTTON_BIT  6                 /* ADC6 */
#elif defined(__AVR_ATmega328PB__)
  #define BUTTON_DDR  DDRE
  #define BUTTON_PORT PORTE
  #define BUTTON_PIN PINE
  #define BUTTON_BIT  2                 /* PE2 / PINE2 */
#else
  #define BUTTON_DDR  DDRB
  #define BUTTON_PORT PORTB
  #define BUTTON_PIN PINB
  #define BUTTON_BIT  4                 /* PB4 */
#endif


/* ======================= Chip-Portabilitaet ======================= */
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328PB__) || defined(__AVR_ATmega328__) \
 || defined(__AVR_ATmega644P__)  || defined(__AVR_ATmega644PA__) \
 || defined(__AVR_ATmega644__)   || defined(__AVR_ATmega644A__) \
 || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega1284__)
  /* 328P / 328PB / 644P(A) / 1284P sind fuer den Booter registergleich (UART0 = USART0, MCUSR,
     TIFR1 an denselben Adressen). Nur der 1284P hat zusaetzlich RAMPZ (in main() auf 0) --
     328P/PB/644P haben <=64 KB Flash, kein RAMPZ. Die Boot-Section-Adresse (BOOT_START) folgt
     generisch aus FLASHEND (328PB: 0x7FFF -> 0x7000, exakt wie 328P). Der 328PB ist ein EIGENER
     Chip (eigenes avr-gcc-Target, eigene Signatur 0x1E9516, Extras USART1/SPI1/TWI1/PORTE), fuer
     den Booter aber 328P-kompatibel -- die Extra-Peripherie ruehrt der Booter nicht an. */
  #define RESET_FLAGS  MCUSR
  #define U_UCSRA UCSR0A
  #define U_UCSRB UCSR0B
  #define U_UCSRC UCSR0C
  #define U_UBRRH UBRR0H
  #define U_UBRRL UBRR0L
  #define U_UDR   UDR0
  #define B_RXC   RXC0
  #define B_UDRE  UDRE0
  #define B_TXC   TXC0
  #define UART_INIT_C()  (U_UCSRC = (1<<UPM01)|(1<<UCSZ01)|(1<<UCSZ00)) /* 8E1 */
  #define T1_IFR  TIFR1              /* Timer1-Overflow-Flag TOV1 sitzt hier */
#elif defined(__AVR_ATmega32A__) || defined(__AVR_ATmega32__)
  #define RESET_FLAGS  MCUCSR
  #define U_UCSRA UCSRA
  #define U_UCSRB UCSRB
  #define U_UDR   UDR
  #define U_UBRRH UBRRH
  #define U_UBRRL UBRRL
  #define B_RXC   RXC
  #define B_UDRE  UDRE
  #define B_TXC   TXC
  /* 32A: UCSRC & UBRRH teilen die Adresse -> Schreiben mit URSEL=1 fuer UCSRC. 8E1. */
  #define UART_INIT_C()  (UCSRC = (1<<URSEL)|(1<<UPM1)|(1<<UCSZ1)|(1<<UCSZ0))
  #define T1_IFR  TIFR               /* 32A: ein gemeinsames Timer-Interrupt-Flag-Register */
#else
  #error "Nicht unterstuetzte MCU (nur ATmega32A / ATmega328P / ATmega328PB / ATmega644P(A) / ATmega1284P)"
#endif

/* ======================= Frame-Konstanten ======================= */
#define FRAME_START       0xFD
#define FRAME_START_SHORT 0xFE
#define ESCAPE_BYTE       0xFC
#define CRC16_POLY        0x1002
#define MAX_PACKET_SIZE   64
#define RX_BUFSIZE        80

enum { CMD_ANNOUNCE='A', CMD_ZERO_START='z', CMD_ZERO_END='Z',
       CMD_START_BOOTER='u', CMD_GET_PACKET_SIZE='p', CMD_WRITE_FLASH='w',
       CMD_READ_FLASH='r', CMD_START_FW='g', CMD_GET_FW='v', CMD_GET_HW='h',
       CMD_GET_SERIAL='n' };

static uint32_t ownAddr;
static uint8_t  rxSenderNum = 0;

/* ======================= UART ======================= */
static void uartInit(void){
  uint16_t ubrr = (uint16_t)(F_CPU/(16UL*BUS_BAUD) - 1);
  U_UBRRH = (uint8_t)(ubrr>>8);
  U_UBRRL = (uint8_t)ubrr;
  UART_INIT_C();
  U_UCSRB = (1<<3)|(1<<4);           /* TXEN|RXEN (Bit3=TXEN, Bit4=RXEN bei beiden MCUs) */
#if USE_DE
  DE_DDR |= (1<<DE_BIT);
  DE_PORT &= ~(1<<DE_BIT);           /* Empfang */
#endif
}
static void deTx(uint8_t on){
#if USE_DE
  if(on) DE_PORT |= (1<<DE_BIT); else DE_PORT &= ~(1<<DE_BIT);
#else
  (void)on;
#endif
}
static void uartPut(uint8_t b){
  while(!(U_UCSRA & (1<<B_UDRE)));
  U_UDR = b;
}
static void uartFlush(void){        /* warten bis letztes Byte komplett raus ist */
  while(!(U_UCSRA & (1<<B_TXC)));
  U_UCSRA |= (1<<B_TXC);
}
static int16_t uartGet(void){       /* -1 wenn nichts da */
  if(U_UCSRA & (1<<B_RXC)) return U_UDR;
  return -1;
}

/* ======================= CRC (aus Sim) ======================= */
static void crc16Shift(uint8_t b, uint16_t* crc){
  for(uint8_t i=0;i<8;i++){
    uint8_t hi = (*crc & 0x8000)?1:0;
    *crc <<= 1;
    if(b & 0x80) *crc |= 1;
    if(hi)       *crc ^= CRC16_POLY;
    b <<= 1;
  }
}

/* ======================= Wire TX (aus Sim) ======================= */
static void txByteEsc(uint8_t b, uint16_t* crc){
  crc16Shift(b, crc);
  if(b==FRAME_START || b==FRAME_START_SHORT || b==ESCAPE_BYTE){
    uartPut(ESCAPE_BYTE); uartPut((uint8_t)(b & 0x7F));
  } else uartPut(b);
}
static void txRawEsc(uint8_t b){
  if(b==FRAME_START || b==FRAME_START_SHORT || b==ESCAPE_BYTE){
    uartPut(ESCAPE_BYTE); uartPut((uint8_t)(b & 0x7F));
  } else uartPut(b);
}
/* control muss das hasSender-Bit (0x08) korrekt gesetzt haben */
static void sendFrame(uint32_t target, uint8_t control, const uint8_t* data, uint8_t len){
  uint8_t hasSender = (control & 0x08);
  uint16_t crc = 0xFFFF;
  deTx(1);
  uartPut(FRAME_START); crc16Shift(FRAME_START,&crc);
  txByteEsc((target>>24)&0xFF,&crc); txByteEsc((target>>16)&0xFF,&crc);
  txByteEsc((target>>8)&0xFF,&crc);  txByteEsc(target&0xFF,&crc);
  txByteEsc(control,&crc);
  if(hasSender){
    txByteEsc((ownAddr>>24)&0xFF,&crc); txByteEsc((ownAddr>>16)&0xFF,&crc);
    txByteEsc((ownAddr>>8)&0xFF,&crc);  txByteEsc(ownAddr&0xFF,&crc);
  }
  txByteEsc(len+2,&crc);
  for(uint8_t i=0;i<len;i++) txByteEsc(data[i],&crc);
  crc16Shift(0,&crc); crc16Shift(0,&crc);
  txRawEsc((crc>>8)&0xFF); txRawEsc(crc&0xFF);
  uartFlush(); deTx(0);
}
static void sendAck(uint32_t to){ sendFrame(to, 0x19 | (uint8_t)(rxSenderNum<<5), 0, 0); }
static void sendInfo(uint32_t to, const uint8_t* d, uint8_t len){
  sendFrame(to, 0x98 | (uint8_t)(rxSenderNum<<5), d, len);
}
/* Bootloader-Antwort auf p/w/r: Die native OpenCCU-hs485d wertet im Zustand WAIT_ACK nur ein
   ACK-Frame (control & 0x97 == 0x11) als "Kommando erledigt" (ACKED). Ein Info-/system-Frame
   bliebe in WAIT_ACK haengen -> WaitUntilSent laeuft in den response-timeout -> Update bricht ab.
   Also ein ACK-Frame (CTRL_ACK 0x19 | Sendefolgenummer<<5) MIT angehaengter Payload senden --
   die hs485d liest die Nutzdaten ueber ExtractFrame().GetPayload(). (0xFD-Standardframe; die
   frueher vermutete 0xFE-system-Form war falsch, OpenCCU-WriteFlash prueft nur size()==2.) */
static void sendBootReply(uint32_t to, const uint8_t* d, uint8_t len){
  sendFrame(to, 0x19 | (uint8_t)(rxSenderNum<<5), d, len);
}
static void makeSerial(uint32_t a, uint8_t* buf){
  buf[0]='H'; buf[1]='B'; buf[2]='W';
  for(int8_t p=9;p>2;p--){ buf[p]='0'+(a%10); if(a) a/=10; }
}
static void sendAnnounce(void){
  uint8_t d[16];
  d[0]=CMD_ANNOUNCE; d[1]=0; d[2]=DEVICE_TYPE; d[3]=HW_VERSION;
  d[4]=(FW_VERSION>>8)&0xFF; d[5]=FW_VERSION&0xFF;
  makeSerial(ownAddr,&d[6]);
  sendFrame(0xFFFFFFFFUL, 0xF8, d, 16);
}
static void sendStartupReason(void){
  uint8_t d[2]={ 0xFF, 0x08 };       /* 0xFF=STARTUP_REASON, 0x08=gewollter Reset */
  sendFrame(0xFFFFFFFFUL, 0xF8, d, 2);
}

/* ======================= Wire RX (aus Sim) ======================= */
static uint8_t  rxb[RX_BUFSIZE];
static uint8_t  rxIdx=0, rxHeaderLen=0, rxHasSender=0;
static int16_t  rxTotal=-1;
static uint8_t  inFrame=0, pendingEsc=0;
static uint16_t rxCrc=0xFFFF;
static void rxReset(void){ inFrame=0; pendingEsc=0; rxIdx=0; rxTotal=-1; rxHeaderLen=0; }

/* Fertig empfangenes, CRC-geprueftes Frame (von rxByte gefuellt, von pollFrame ausgeliefert).
   Eigene Variablen statt Rueckgabe-Pointer, weil ein Frame auch WAEHREND einer Flash-Operation
   fertig werden kann (rxPump aus spmWait heraus) und dann bis zur Auswertung liegenbleiben muss. */
static uint8_t  frameReady=0;
static uint32_t fTarget, fSender;
static uint8_t  fControl, *fData, fDlen;

/* Ein einzelnes Byte in die Statemaschine schieben. 1 = komplettes gueltiges Frame liegt vor. */
static uint8_t rxByte(uint8_t b){
  {
    if(b==FRAME_START || b==FRAME_START_SHORT){
      inFrame=1; pendingEsc=0; rxIdx=0; rxTotal=-1; rxHeaderLen=0;
      rxCrc=0xFFFF; crc16Shift(b,&rxCrc); return 0;
    }
    if(!inFrame) return 0;
    if(b==ESCAPE_BYTE){ pendingEsc=1; return 0; }
    if(pendingEsc){ b|=0x80; pendingEsc=0; }
    crc16Shift(b,&rxCrc);
    if(rxIdx<RX_BUFSIZE) rxb[rxIdx]=b;
    if(rxIdx==4){
      /* hasSender-Erkennung: An einen BOOTER setzt die CCU/hs485d teils nur Bit 4 (0x10)
         statt Bit 3 (0x08) -- z.B. control 0x12/0x16 statt 0x18. Beide FUEHREN eine
         Senderadresse (per CRC am echten Bus belegt). Daher Bit 3 ODER Bit 4 werten,
         ausser bei Discovery (Bits 1,0 = 0b11, dort ist Bit 3 kein Sender-Flag). Der
         CRC-Check am Frame-Ende faengt eine falsche Annahme ab -> Frame wird verworfen. */
      uint8_t c=b;
      rxHasSender=((c & 0x18)!=0 && (c & 0x03)!=0x03) ? 1 : 0;
      rxHeaderLen=rxHasSender?10:6;
    }
    if(rxHeaderLen && rxIdx==(rxHeaderLen-1)) rxTotal=rxHeaderLen+b;
    rxIdx++;
    if(rxTotal>0 && rxIdx==rxTotal){
      if(rxCrc!=0){ rxReset(); return 0; }
      fTarget=((uint32_t)rxb[0]<<24)|((uint32_t)rxb[1]<<16)|((uint32_t)rxb[2]<<8)|rxb[3];
      fControl=rxb[4];
      fSender = rxHasSender ? (((uint32_t)rxb[5]<<24)|((uint32_t)rxb[6]<<16)|((uint32_t)rxb[7]<<8)|rxb[8]) : 0;
      uint8_t wireLen=rxb[rxHeaderLen-1];
      fDlen=(wireLen>=2)?wireLen-2:0;
      fData=&rxb[rxHeaderLen];
      rxReset(); return 1;
    }
    if(rxIdx>=RX_BUFSIZE) rxReset();
  }
  return 0;
}

/* UART leerraeumen, solange Bytes da sind. Stoppt, sobald ein Frame fertig ist -- sonst wuerden
   nachfolgende Bytes rxb ueberschreiben, auf das fData noch zeigt. Wird AUCH aus spmWait()
   heraus aufgerufen: waehrend boot_page_erase/_write (~4,5-9 ms) liefe der 2-Byte-UART-Puffer
   sonst ueber und der naechste Block ginge verloren (Ursache des Abbruchs an der Page-Grenze). */
static void rxPump(void){
  int16_t v;
  while(!frameReady && (v=uartGet())>=0){
    if(rxByte((uint8_t)v)) frameReady=1;
  }
}

/* Warten auf das Ende einer SPM-Operation -- dabei WEITER die UART bedienen.
   Zulaessig, weil der Booter in der NRWW-Section liegt: die CPU laeuft waehrend eines
   Schreibvorgangs in die RWW-(App-)Section normal weiter, und rxByte arbeitet nur auf RAM. */
static void spmWait(void){
  while(boot_spm_busy()) rxPump();
}

static uint8_t pollFrame(uint32_t* target, uint8_t* control, uint32_t* sender,
                         uint8_t** data, uint8_t* dlen){
  rxPump();
  if(!frameReady) return 0;
  frameReady=0;
  *target=fTarget; *control=fControl; *sender=fSender; *data=fData; *dlen=fDlen;
  return 1;
}

/* ======================= Status-LED ======================= */
static void ledInit(void){
#if USE_LED
  LED_DDR |= (1<<LED_BIT);
#endif
}
static void ledSet(uint8_t on){
#if USE_LED
 #if LED_ACTIVE_LOW
  on = !on;
 #endif
  if(on) LED_PORT |=  (1<<LED_BIT);
  else   LED_PORT &= ~(1<<LED_BIT);
#else
  (void)on;
#endif
}

/* ======================= Config-Taster ======================= */
/* Config-Taster gedrueckt halten beim reset, zum erzwingen des booters */
static uint8_t configPressed(void){
#if USE_BUTTON
 #if (defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__)) && defined BUTTON_ADC_ONLY && BUTTON_BIT >= 6
/* Config-Taster an ADC6 oder 7 abfragen (analog only ports ADC6 & 7, kein digitales IO) */
  ADMUX  = (1<<REFS0) | BUTTON_BIT;                                  /* AVcc, Kanal ADC6 */
  ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);      /* ADC an, /128 */
  ADCSRA |= (1<<ADSC); while(ADCSRA & (1<<ADSC));           /* Dummy-Wandlung */
  ADCSRA |= (1<<ADSC); while(ADCSRA & (1<<ADSC));           /* echte Messung */
  uint16_t v = ADC;
  ADCSRA = 0;                                               /* ADC wieder aus */
  return (v < 250);                                         /* gedrueckt = LOW */
 #else
  // IO port nutzen
  BUTTON_DDR &= ~(1<<BUTTON_BIT);        /* IO als Eingang setzen (zur Sicherheit falls es kein komplett reset war) */
  BUTTON_PORT |=  (1<<BUTTON_BIT);   /* interner Pull-up */
  _delay_us(10);                     /* einschwingen lassen */
  return !(BUTTON_PIN & (1<<BUTTON_BIT));
 #endif
#else
  return 0;
#endif
}

/* ======================= App starten (jmp 0) ======================= */
static void startApp(void){
  ledSet(0);                         /* LED aus -- ab hier gehoert sie der App */
  U_UCSRB=0;                         /* UART aus */
  cli();
  ((void(*)(void))0)();              /* Sprung zum Reset-Vektor der App */
}

/* ======================= Flash schreiben (avr/boot.h) ======================= */
/* Booter-Section-Start (Byte-Adresse) = oberste 4 KB (2048 Words, BOOTSZ=00) des Flash,
   generisch aus FLASHEND: 32A/328P 0x7000 (32 KB), 644P/644PA 0xF000 (64 KB), 1284P 0x1F000
   (128 KB). Der Booter darf sich hier NIE selbst ueberschreiben. Muss zur Boot-Adresse im
   build.sh-Linkerflag passen (beide = FLASHEND + 1 - 0x1000). */
#define BOOT_START ((uint32_t)(FLASHEND + 1UL - 0x1000UL))
static uint8_t  pageBuf[SPM_PAGESIZE];
static uint16_t pageBase = 0xFFFF;
static uint8_t  pageDirty = 0;
static uint8_t  flashStarted = 0;    /* erst beim ersten 'w' Page 0 (Reset-Vektor) invalidieren */
static uint8_t  ledBusy = 0;         /* ab dem ersten 'w'/'r': LED auf "Update laeuft" umschalten.
                                        Eigenes Flag, weil 'p' flashStarted zuruecksetzt (hs485d
                                        schickt 'p' auch am Verify-Start) -- die LED soll waehrend
                                        des Verifys aber weiter "beschaeftigt" zeigen. */

static void writePage(uint16_t base){
  boot_page_erase(base); spmWait();
  for(uint16_t i=0;i<SPM_PAGESIZE;i+=2)
    boot_page_fill(base+i, pageBuf[i] | ((uint16_t)pageBuf[i+1]<<8));
  boot_page_write(base); spmWait();
  boot_rww_enable();                 /* App-Section wieder lesbar machen */
}
static void flushPage(void){ if(pageDirty){ writePage(pageBase); pageDirty=0; } }
static void bufferBytes(uint16_t addr, const uint8_t* src, uint8_t n){
  for(uint8_t i=0;i<n;i++){
    uint16_t a=addr+i, pb=a & ~(SPM_PAGESIZE-1);
    if(pb!=pageBase){ flushPage(); pageBase=pb; memset(pageBuf,0xFF,SPM_PAGESIZE); }
    pageBuf[a-pageBase]=src[i]; pageDirty=1;
  }
}
static uint16_t appCrc(uint16_t len){          /* CRC16 (Poly 0x1002) ueber App-Flash 0..len-1 */
  uint16_t crc=0xFFFF;
  for(uint16_t a=0;a<len;a++) crc16Shift(pgm_read_byte(a), &crc);
  return crc;
}

/* Blinkmuster aus dem frei laufenden Timer1 (clk/1024 = 15625 Hz @16 MHz) ableiten -- ohne
   delay(), damit das Bus-Timing unberuehrt bleibt. Wird in jedem Schleifendurchlauf aufgerufen. */
static void ledUpdate(void){
#if USE_LED
  uint16_t t = TCNT1;
  if(ledBusy) ledSet((t & (1U<<11)) != 0);      /* ~3,8 Hz: Update laeuft */
  else        ledSet((t & 0x3FFF) < 0x0400);    /* 65 ms Blitz je 1,05 s: Booter wartet */
#endif
}

/* ======================= Kommando-Logik ======================= */
static uint8_t zCount=0, zeroComm=0;

static uint8_t handleFrame(uint32_t target, uint8_t control, uint32_t sender,
                        uint8_t* data, uint8_t dlen){
  rxSenderNum=(control>>1)&0x03;
  uint8_t broadcast=(target==0xFFFFFFFFUL);
  if(!broadcast && target!=ownAddr) return 0;
  if(dlen==0) return 0;
  uint8_t cmd=data[0];

  if(cmd==CMD_ZERO_START){ if(zCount>=1) zeroComm=1; else zCount++; return 1; }
  if(cmd==CMD_ZERO_END){ zeroComm=0; zCount=0; return 1; }
  /* Antwortende Kommandos (h/v/n/p/w/r/g/u/...) NUR an die EIGENE Adresse beantworten. Sonst
     antworten bei einem Broadcast alle Booter am Bus gleichzeitig -> Kollision (Thomas' Fund mit
     zweitem Geraet am Bus). z/Z (oben) sind die einzigen Broadcast-Kommandos und bleiben antwortlos. */
  if(broadcast) return 0;
  if(zeroComm && cmd!=CMD_START_BOOTER) return 0;

  switch(cmd){
    case CMD_START_BOOTER:             /* im Booter sind wir schon -> bestaetigen + melden */
      zeroComm=0; zCount=0;
      sendAck(sender);
      sendStartupReason();
      sendAnnounce();
      break;
    case CMD_GET_PACKET_SIZE:{        /* 'p' = Blockgroesse melden. Reset-Vektor (Page 0) hier NOCH
                                          NICHT loeschen -- erst beim ersten echten 'w'. Sonst
                                          zerstoert ein leerer Handshake (p -> g ohne w, z.B. wenn
                                          die CCU kein Firmware-Image hat) die noch intakte App. */
      flushPage();                     /* WICHTIG: offene Page ZUERST committen. hs485d ruft 'p' auch
                                          am VerifyFlash-Start (NACH der w-Schleife) -- die letzte,
                                          noch nicht geschriebene w-Page (z.B. mit Versionsfeld @0x6FF0)
                                          ginge sonst durch das pageDirty=0 unten verloren -> Verify
                                          liest dort 0xFF -> Mismatch. Beim leeren Handshake ist
                                          pageDirty=0 -> no-op, die intakte App bleibt unberuehrt. */
      pageBase=0xFFFF; pageDirty=0; flashStarted=0;   /* Page-Puffer + Flash-Zustand zuruecksetzen */
      uint8_t r[2]={0x00,MAX_PACKET_SIZE};            /* Blockgroesse als 2-Byte-BIG-ENDIAN, wie hs485d es liest */
      sendBootReply(sender,r,2); break; }
    /* h/v/n im APP-Format beantworten (OHNE cmd-Echo im 1. Byte) -- so liest CCU/Gateway die Werte wie
       bei der laufenden App: h=[Typ,HW], v=[FWhi,FWlo], n=[Serial(10)]. Greift nur im reinen Booter-
       Zustand (ohne App); mit DEVICE_TYPE=0x00 taucht so ein Geraet neutral auf, nicht als Fremdtyp. */
    case CMD_GET_FW:{ uint8_t r[2]={(FW_VERSION>>8)&0xFF,FW_VERSION&0xFF}; sendInfo(sender,r,2); break; }
    case CMD_GET_HW:{ uint8_t r[2]={DEVICE_TYPE,HW_VERSION}; sendInfo(sender,r,2); break; }
    case CMD_GET_SERIAL:{ uint8_t r[10]; makeSerial(ownAddr,r); sendInfo(sender,r,10); break; }
    case CMD_WRITE_FLASH:{             /* 'w' [addrHi addrLo len data..] -> App-Section flashen */
      if(dlen>=4){
        uint16_t addr=((uint16_t)data[1]<<8)|data[2];
        uint8_t  n=data[3]; if(n>(uint8_t)(dlen-4)) n=dlen-4;
        /* ACK ZUERST senden -- BEVOR geflasht wird. An 128-B-Page-Grenzen blockiert boot_page_write
           (via flushPage in bufferBytes) ~7 ms; das darf die ACK NICHT verzoegern, sonst laeuft die
           hs485d evtl. in ihren Response-Timeout und wiederholt genau diesen Block. data[] bleibt in
           rxb gueltig, bis das naechste Frame empfangen wird (waehrend sendBootReply blockiert der
           Booter im Senden, empfaengt also nichts). Bestaetigt auch abgelehnte Boot-Section-Bloecke. */
        uint8_t r[2]={0x00,n};
        sendBootReply(sender,r,2);
        ledBusy=1;                   /* Daten fliessen -> LED auf "Update laeuft" */
        /* Nutzdaten JETZT aus rxb herauskopieren: waehrend der folgenden Flash-Operationen laeuft
           spmWait()->rxPump() weiter und schreibt das naechste Frame in rxb -- data[] waere dann
           mitten im Puffern ueberschrieben. */
        uint8_t wbuf[MAX_PACKET_SIZE];
        if(n>MAX_PACKET_SIZE) n=MAX_PACKET_SIZE;
        memcpy(wbuf,&data[4],n);
        if(!flashStarted){             /* ERSTER Datenblock: Reset-Vektor (Page 0) invalidieren ->
                                          ein Abbruch laesst die App garantiert ungueltig. */
          boot_page_erase(0); spmWait(); boot_rww_enable();
          flashStarted=1;
        }
        if((uint32_t)addr+n <= BOOT_START) bufferBytes(addr,wbuf,n);      /* Boot-Section schuetzen */
      }
      break; }
    case CMD_READ_FLASH:{              /* 'r' [addrHi addrLo len] -> echte Flash-Bytes (Verify) */
      if(dlen>=4){ uint16_t addr=((uint16_t)data[1]<<8)|data[2]; uint8_t n=data[3];
        ledBusy=1;                     /* Verify laeuft auch als "Update" anzeigen */
        flushPage();                   /* offene Page committen -- ERST NACH dem Auslesen von addr/n:
                                          flushPage kann flashen, dabei laeuft rxPump() und
                                          ueberschreibt data[] (zeigt in rxb). */
        if(n>MAX_PACKET_SIZE)n=MAX_PACKET_SIZE;
        /* Payload = GENAU die n gelesenen Bytes, KEIN cmd/addr/len-Echo davor. hs485d VerifyFlash
           verwirft die Antwort sonst hart: `if(response.size()!=blocksize)return false` -> die CCU
           meldet "unbekannter Fehler", obwohl der Flash korrekt ist (g laeuft, App startet). Zuordnung
           laeuft ueber den frameCounter, nicht ueber den Payload-Inhalt -> Echo ist unnoetig. */
        uint8_t r[MAX_PACKET_SIZE];
        for(uint8_t i=0;i<n;i++) r[i]=pgm_read_byte(addr+i);
        sendBootReply(sender,r,n); }
      break; }
    case CMD_START_FW:{                /* 'g' [lenHi lenLo crcHi crcLo] -> CRC-Check + App-Start */
      /* Parameter ZUERST sichern -- flushPage()/appCrc() flashen bzw. lassen rxPump() laufen. */
      uint16_t len=0, want=0; uint8_t haveCrc=(dlen>=5);
      if(haveCrc){ len =((uint16_t)data[1]<<8)|data[2];
                   want=((uint16_t)data[3]<<8)|data[4]; }
      flushPage();
      sendAck(sender);
      _delay_ms(5);
      if(haveCrc){                     /* mit erwarteter CRC: echte Validierung der ganzen App */
        if(appCrc(len)==want) startApp();
      } else if(pgm_read_word(0)!=0xFFFF){ /* ohne CRC: schwacher Reset-Vektor-Sanity */
        startApp();
      }
      break; }                         /* CRC falsch / unvollstaendig -> im Booter bleiben */
    case CMD_ANNOUNCE: sendAnnounce(); break;
    default: sendAck(sender); return 0; break;   /* im Booter alles ACKen, damit die CCU weiterlaeuft */
  }
  return 1;                 /* ok für alle anderen gültigen CMDs */
}

/* ======================= main ======================= */
int main(void){
  uint8_t rf = RESET_FLAGS;           /* Reset-Quelle sichern ... */
  RESET_FLAGS = 0;                    /* ... und Flags loeschen */
  wdt_disable();                      /* PFLICHT nach WDRF: sonst Reset-Loop */
#ifdef RAMPZ
  RAMPZ = 0;                          /* 1284P: App <64 KB -> LPM/SPM adressieren die untere
                                         Flash-Haelfte ohne Bank; der Booter macht nie _far-Zugriffe. */
#endif
  /* RAM-Marker der App auslesen und SOFORT entwerten (siehe bootmagic.h). Nur ein gewolltes
   * 'u'-Update setzt ihn; '!!'-Reset / App-Restart / Watchdog-Hang / Power-on tun das nicht. */
  uint8_t updateWanted = (BOOT_MAGIC_CELL == BOOT_MAGIC_VAL);
  BOOT_MAGIC_CELL = 0;                 /* entwerten: ein spaeterer Reset OHNE neues 'u' bleibt Reset */
  uint8_t NoApp = (pgm_read_word(0) == 0xFFFF);
  if (configPressed()) NoApp = 1;      /* bei gedruektem Konfig Taster im Booter bleiben. NoApp ueberschreiben */
  /* Reset-QUELLEN-Entscheidung, WDRF-Mehrdeutigkeit per Marker aufgeloest:
   * - kein WDRF (Power-on/Brown-out/extern) + App    -> App
   * - WDRF OHNE Marker ('!!'/Restart/WDT-Hang) + App -> App  (frueher blieb er faelschlich im Booter)
   * - WDRF UND Marker ('u' Update gewollt)           -> im Booter bleiben
   * - keine gueltige App (Reset-Vektor 0xFFFF)       -> immer im Booter (Flash unfertig)
   * ACHTUNG (von loetmeister aufgezeigt): Der Marker ist damit PFLICHT. Eine App OHNE
   * Marker-Patch setzt ihn nie -> !updateWanted ist immer wahr -> die Oder-Bedingung ist immer
   * wahr -> startApp() sofort: so eine App kommt NIE in den Booter (und der Erst-Timeout unten
   * wird gar nicht erst erreicht -- er ist KEIN Fallback dafuer). Bewusst so: bus-updatefaehig
   * ist nur, wer mit der gepatchten HBWired-Lib gebaut ist.
   * Der Timeout (idleOvf) wirkt bei ABGEBROCHENEM Update oder configPressed: Booter korrekt per
   * Marker betreten, dann bleibt der Bus still und die App ist noch intakt -> zurueck in die App. */
  if(!NoApp && (!(rf & (1<<WDRF)) || !updateWanted)){
    startApp();
  }

  /* Bus-Adresse aus den letzten 4 EEPROM-Bytes (E2END-3), big-endian */
  {
    uint16_t a = E2END-3;
    ownAddr = ((uint32_t)eeprom_read_byte((uint8_t*)a)<<24)
            | ((uint32_t)eeprom_read_byte((uint8_t*)(a+1))<<16)
            | ((uint32_t)eeprom_read_byte((uint8_t*)(a+2))<<8)
            |  (uint32_t)eeprom_read_byte((uint8_t*)(a+3));
    if(ownAddr == 0xFFFFFFFFUL) ownAddr = FALLBACK_ADDR;
  }

  uartInit();
  ledInit();
  _delay_ms(20);                      /* Bus kurz beruhigen */
  sendStartupReason();
  _delay_ms(5);
  sendAnnounce();

  /* Inaktivitaets-Timeout gegen "im Booter haengen bleiben": Timer1 frei laufen
   * lassen (Prescaler 1024 -> ~4,19 s je Ueberlauf @16 MHz). Jeder empfangene Frame
   * setzt den ZAEHLER idleOvf zurueck -- TCNT1 selbst laeuft bewusst DURCH, weil die
   * Status-LED ihr Blinkmuster daraus ableitet (ein Reset bei jedem Frame liesse sie
   * waehrend eines Updates stehen). Kostet nur Granularitaet: der Timeout greift dadurch
   * nach 21-25 s statt exakt 25 s. Bleibt der Bus IDLE_TIMEOUT_OVF Ueberlaeufe lang still UND ist
   * die App intakt (Reset-Vektor != 0xFFFF), springen wir zurueck in die App. Waehrend
   * eines echten Flashs ist Page 0 geloescht -> Reset-Vektor 0xFFFF -> der Timeout
   * startet dann NICHTS (halbe App bleibt liegen), und laufende w-Bloecke halten ihn
   * ohnehin frisch. Er wirkt also nur beim echten Haenger. */
  TCCR1A = 0;
  TCCR1B = (1<<CS12)|(1<<CS10);        /* Timer1: clk/1024, Normal-Mode */
  TCNT1  = 0; T1_IFR = (1<<TOV1);
  uint8_t idleOvf = 0;

  for(;;){
    uint32_t target,sender; uint8_t control,*data,dlen;
    ledUpdate();
    if(pollFrame(&target,&control,&sender,&data,&dlen)){
      if (handleFrame(target,control,sender,data,dlen)) idleOvf = 0;       /* Aktivitaet -> Timeout zuruecksetzen */
    }
    if(T1_IFR & (1<<TOV1)){             /* ~4,19 s vergangen */
      T1_IFR = (1<<TOV1);
      if(++idleOvf >= IDLE_TIMEOUT_OVF && pgm_read_word(0) != 0xFFFF)
        startApp();                    /* lange still + App intakt -> zurueck in die App */
    }
  }
}
