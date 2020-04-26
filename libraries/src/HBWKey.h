#ifndef HBWKey_h
#define HBWKey_h

#include <inttypes.h>
#include "HBWired.h"


//#define DEBUG_OUTPUT

#define KEY_DEBOUNCE_TIME 65  // ms
#define SWITCH_DEBOUNCE_TIME 145  // ms
#define DOORSENSOR_DEBOUNCE_TIME 330  // ms


// below values have to match INPUT_TYPE in XML!
#define IN_DOORSENSOR   0    // sends a short KeyEvent on HIGH and long KeyEvent on LOW input level changes
#define IN_MOTIONSENSOR 1    // sends a short KeyEvent for raising or falling edge - not both
#define IN_SWITCH       2    // sends a short KeyEvent, each time the input (e.g. wall switch) changes the polarity
#define IN_PUSHBUTTON   3    // sends a short KeyEvent on short press and (repeated) long KeyEvent on long press


// config, address step 2
struct hbw_config_key {
  uint8_t n_input_locked:1;   // 0x07:0    0=LOCKED, 1=UNLOCKED
  uint8_t n_inverted:1;       // 0x07:1    0=inverted, 1=not inverted
  uint8_t pullup:1;           // 0x07:2 // TODO: needed....?
  uint8_t input_type:2;       // 0x07:3-4   3=PUSHBUTTON (default), 2=SWITCH, 1=MOTIONSENSOR, 0=DOORSENSOR?
  uint8_t       :3;           // 0x07:5-7
  uint8_t long_press_time;    // 0x08
};


// Class HBWKey
class HBWKey : public HBWChannel {
  public:
    HBWKey(uint8_t _pin, hbw_config_key* _config, boolean _activeHigh = false);
    virtual void loop(HBWDevice*, uint8_t channel);
    virtual void afterReadConfig();

  private:
    uint8_t pin;   // Pin
    uint32_t keyPressedMillis;  // Zeit, zu der die Taste gedrueckt wurde (fuer's Entprellen)
    uint32_t lastSentLong;      // Zeit, zu der das letzte Mal longPress gesendet wurde
    uint8_t keyPressNum;
    hbw_config_key* config;
    boolean oldButtonState;
    boolean activeHigh;    // activeHigh=true -> input active high, else active low
};

#endif
