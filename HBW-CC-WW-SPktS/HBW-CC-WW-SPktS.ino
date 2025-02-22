//*******************************************************************
//
// HBW-CC-WW-SPktS
//
// Homematic Wired Hombrew Hardware
// Arduino NANO als Homematic-Device
// Schwingungspaketsteuerung (Zero-crossing control) mit Zusatzfunktionen (DeltaT Regler & OneWire Temperatursensoren)
// - Direktes Peering für Temperatursensoren und DeltaTx Eingänge
//
// http://loetmeister.de/Elektronik/homematic/index.htm#modules
//
//*******************************************************************
// Changes
// v0.01
// - initial version
// v0.02
// - added option to pulse on time for delta T output channel (50% duty cycle)


#define HARDWARE_VERSION 0x01
#define FIRMWARE_VERSION 0x0002
#define HMW_DEVICETYPE 0x99 //device ID (make sure to import hbw-cc-ww-spkts.xml into FHEM)

#define NUMBER_OF_HEATING_CHAN 1   // Schwingungspaketsteuerungsausgangskanal
#define NUMBER_OF_HEAT_DELTAT_CHAN NUMBER_OF_HEATING_CHAN  // dT1 channel linked with heating/dimmer chan above
#define NUMBER_OF_TEMP_CHAN 5   // input channels - 1-wire temperature sensors
#define NUM_LINKS_TEMP 32    // requires Support_HBWLink_InfoEvent in HBWired.h
#define NUMBER_OF_DELTAT_CHAN 2 // result output channels[, can peer with switch]
#define NUM_LINKS_DELTATX NUMBER_OF_DELTAT_CHAN*2 +NUMBER_OF_HEAT_DELTAT_CHAN  // allow to peer input channels (T1 & T2) with one temperature sensor each
#define ADDRESS_START_CONF_TEMP_CHAN 0x0C  // first EEPROM address for temperature sensors configuration
#define LINKADDRESSSTART_TEMP 0x100  // pering start_address for any sensor type peers, address_step has to be 6
#define LINKADDRESSSTART_DELTATX 0x220  // step 7, actor type


// HB Wired protocol and module
#include <HBWired.h>
#include <HBWOneWireTempSensors.h>
#include <HBWLinkInfoEventSensor.h>
#include <HBWLinkInfoEventActuator.h>
#include "HBWDeltaT.h"
#include "HBWSPktS.h"
#include <HBW_eeprom.h>


// Pins and hardware config
#include "HBW-CC-WW-SPktS_config_example.h"  // When using custom device pinout or controller, copy this file and include it instead


#define NUMBER_OF_CHAN NUMBER_OF_HEATING_CHAN + NUMBER_OF_TEMP_CHAN + (NUMBER_OF_DELTAT_CHAN *3) + NUMBER_OF_HEAT_DELTAT_CHAN


struct hbw_config
{
  uint8_t logging_time;     // 0x01
  uint32_t central_address;  // 0x02 - 0x05
  uint8_t direct_link_deactivate:1;   // 0x06:0
  uint8_t              :7;   // 0x06:1-7
  hbw_config_dim_spkts SPktSCfg[NUMBER_OF_HEATING_CHAN];  // 0x07 - 0x0A (address step 4)
  hbw_config_DeltaTx Temp1Cfg[NUMBER_OF_HEATING_CHAN];    // 0x0B - 0x0B (address step 1)
  hbw_config_onewire_temp TempOWCfg[NUMBER_OF_TEMP_CHAN]; // 0x0C - 0x51 (address step 14)
  hbw_config_DeltaT DeltaTCfg[NUMBER_OF_DELTAT_CHAN];     // 0x52 - 0x59 (address step 7)
  hbw_config_DeltaTx DeltaT1Cfg[NUMBER_OF_DELTAT_CHAN];   // 0x60 - 0x61 (address step 1)
  hbw_config_DeltaTx DeltaT2Cfg[NUMBER_OF_DELTAT_CHAN];   // 0x62 - 0x63 (address step 1)
} hbwconfig;


HBWChannel* channels[NUMBER_OF_CHAN];  // total number of channels for the device

// global pointer for OneWire channels
hbw_config_onewire_temp* tempConfig[NUMBER_OF_TEMP_CHAN]; // pointer for config


class HBDCCDevice : public HBWDevice
{
    public:
    HBDCCDevice(uint8_t _devicetype, uint8_t _hardware_version, uint16_t _firmware_version,
               Stream* _rs485, uint8_t _txen, 
               uint8_t _configSize, void* _config, 
               uint8_t _numChannels, HBWChannel** _channels,
               Stream* _debugstream, HBWLinkSender* linksender = NULL, HBWLinkReceiver* linkreceiver = NULL,
               OneWire* oneWire = NULL, hbw_config_onewire_temp** _tempSensorconfig = NULL) :
    HBWDevice(_devicetype, _hardware_version, _firmware_version,
              _rs485, _txen, _configSize, _config, _numChannels, ((HBWChannel**)(_channels)),
              _debugstream, linksender, linkreceiver)
              {
                d_ow = oneWire;
                tempSensorconfig = _tempSensorconfig;
    };
    virtual void afterReadConfig();
    
    private:
      OneWire* d_ow;
      hbw_config_onewire_temp** tempSensorconfig;
};

// device specific defaults
void HBDCCDevice::afterReadConfig()
{
  if(hbwconfig.logging_time == 0xFF) hbwconfig.logging_time = 50;

  HBWOneWireTemp::sensorSearch(d_ow, tempSensorconfig, (uint8_t) NUMBER_OF_TEMP_CHAN, (uint8_t) ADDRESS_START_CONF_TEMP_CHAN);
};

HBDCCDevice* device = NULL;

// global vars
volatile unsigned char SPktS_currentValue;  // Schwingungspaketsteuerung, Anzahl "Ein" Wellen, basierend auf Sollwert
uint8_t SPktS_outputPin;
// TODO: put into hpp?
// TODO: allow more channels? Use arrays SPktS_outputPin[], WellenpaketCnt[], SPktS_currentValue[] ? ... and loop in ISR?

ISR(TIMER1_COMPA_vect)
{
  //timer1 interrupt 50Hz

  static byte WellenpaketCnt = 1;

  if (SPktS_currentValue && WellenpaketCnt <= SPktS_currentValue) digitalWrite(SPktS_outputPin, HIGH);
  else  digitalWrite(SPktS_outputPin, LOW);

  if (WellenpaketCnt < WELLENPAKETSCHRITTE) WellenpaketCnt++;
  else WellenpaketCnt = 1;
}

void setup()
{
  // variables for all OneWire channels
  uint32_t g_owLastReadTime = 0;
  uint8_t g_owCurrentChannel = OW_CHAN_INIT; // always initialize with OW_CHAN_INIT value!
  OneWire* g_ow = new OneWire(ONEWIRE_PIN);
  
  // create channels
  for(uint8_t i = 0; i < NUMBER_OF_TEMP_CHAN; i++) {
    channels[i +NUMBER_OF_HEATING_CHAN *2] = new HBWOneWireTemp(g_ow, &(hbwconfig.TempOWCfg[i]), &g_owLastReadTime, &g_owCurrentChannel);
    tempConfig[i] = &(hbwconfig.TempOWCfg[i]);
  }

  static const uint8_t relayPin[NUMBER_OF_DELTAT_CHAN] = {RELAY_1, RELAY_2};  // assing pins
  HBWDeltaTx* deltaTxCh[(NUMBER_OF_DELTAT_CHAN *2) +NUMBER_OF_HEAT_DELTAT_CHAN];  // pointer to deltaTx channels, to link in deltaT channels

#if NUMBER_OF_DELTAT_CHAN == 2 && NUMBER_OF_TEMP_CHAN == 5
  for(uint8_t i = 0; i < NUMBER_OF_DELTAT_CHAN; i++) {
    deltaTxCh[i] = new HBWDeltaTx( &(hbwconfig.DeltaT1Cfg[i]) );
    deltaTxCh[i +NUMBER_OF_DELTAT_CHAN] = new HBWDeltaTx( &(hbwconfig.DeltaT2Cfg[i]) );
    channels[i +NUMBER_OF_HEATING_CHAN*2 +NUMBER_OF_TEMP_CHAN] = new HBWDeltaT(relayPin[i], deltaTxCh[i], deltaTxCh[i +NUMBER_OF_DELTAT_CHAN], &(hbwconfig.DeltaTCfg[i]));
    channels[i +NUMBER_OF_HEATING_CHAN*2 +NUMBER_OF_TEMP_CHAN +NUMBER_OF_DELTAT_CHAN] = deltaTxCh[i];
    channels[i +NUMBER_OF_HEATING_CHAN*2 +NUMBER_OF_TEMP_CHAN +NUMBER_OF_DELTAT_CHAN *2] = deltaTxCh[i +NUMBER_OF_DELTAT_CHAN];
  }
  deltaTxCh[(NUMBER_OF_DELTAT_CHAN *2) +NUMBER_OF_HEAT_DELTAT_CHAN -1] = new HBWDeltaTx( &(hbwconfig.Temp1Cfg[0]) );
#else
  #error deltaT channel count and pin missmatch!
#endif

  SPktS_outputPin = HEATER_PIN;
  channels[0] = new HBWSPktS(&SPktS_outputPin, &(hbwconfig.SPktSCfg[0]), deltaTxCh[(NUMBER_OF_DELTAT_CHAN *2) +NUMBER_OF_HEAT_DELTAT_CHAN -1], &SPktS_currentValue);
  channels[1] = deltaTxCh[(NUMBER_OF_DELTAT_CHAN *2) +NUMBER_OF_HEAT_DELTAT_CHAN -1];


#ifdef USE_HARDWARE_SERIAL  // RS485 via UART Serial, no debug (_debugstream is NULL)
  Serial.begin(19200, SERIAL_8E1);
  
  device = new HBDCCDevice(HMW_DEVICETYPE, HARDWARE_VERSION, FIRMWARE_VERSION,
                             &Serial, RS485_TXEN, sizeof(hbwconfig), &hbwconfig,
                             NUMBER_OF_CHAN, (HBWChannel**)channels,
                             NULL,
                             new HBWLinkInfoEventSensor<NUM_LINKS_TEMP, LINKADDRESSSTART_TEMP>(),
                             new HBWLinkInfoEventActuator<NUM_LINKS_DELTATX, LINKADDRESSSTART_DELTATX>(),
                             g_ow, tempConfig
                             );
  
  device->setConfigPins(BUTTON, LED);  // use analog input for 'BUTTON'
  
#else
  Serial.begin(115200);  // Serial->USB for debug
  rs485.begin(19200);   // RS485 via SoftwareSerial, must use 19200 baud!
  
  device = new HBDCCDevice(HMW_DEVICETYPE, HARDWARE_VERSION, FIRMWARE_VERSION,
                             &rs485, RS485_TXEN, sizeof(hbwconfig), &hbwconfig,
                             NUMBER_OF_CHAN, (HBWChannel**)channels,
                             &Serial,
                             new HBWLinkInfoEventSensor<NUM_LINKS_TEMP, LINKADDRESSSTART_TEMP>(),
                             new HBWLinkInfoEventActuator<NUM_LINKS_DELTATX, LINKADDRESSSTART_DELTATX>(),
                             g_ow, tempConfig
                             );
  
  device->setConfigPins(BUTTON, LED);  // 8 (button) and 13 (led) is the default

  hbwdebug(F("B: 2A "));
  hbwdebug(freeRam());
  hbwdebug(F("\n"));
#endif


setup_timer();  // finally configure and enable the timer interrupt
// // Init timer ITimer1
// ITimer2.init();
// if (ITimer2.attachInterruptInterval(TIMER_INTERVAL_MS, TimerHandler, HEATER_PIN)) hbwdebug(F("TimerISR: ok"));
}


void loop()
{
  device->loop();
  POWERSAVE();  // go sleep a bit
};
