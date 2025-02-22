/* 
* HBWSwitchAdvanced.cpp
*
* Als Alternative zu HBWSwitch & HBWLinkSwitchSimple sind mit
* HBWSwitchAdvanced & HBWLinkSwitchAdvanced folgende Funktionen möglich:
* Peering mit TOGGLE, TOGGLE_TO_COUNTER, TOGGLE_INVERSE_TO_COUNTER, onTime,
* offTime (Ein-/Ausschaltdauer), onDelayTime, offDelayTime (Ein-/Ausschaltverzögerung).
*
* http://loetmeister.de/Elektronik/homematic/
*
*/

#include "HBWSwitchAdvanced.h"

// Switches class
HBWSwitchAdvanced::HBWSwitchAdvanced(uint8_t _pin, hbw_config_switch* _config) {
  pin = _pin;
  config = _config;
  clearFeedback();
  init();
};


// channel specific settings or defaults
void HBWSwitchAdvanced::afterReadConfig()
{
  if (pin == NOT_A_PIN)  return;
  
  if (currentState == UNKNOWN_STATE) {
  // All off on init, but consider inverted setting
    digitalWrite(pin, config->n_inverted ? LOW : HIGH);    // 0=inverted, 1=not inverted (device reset will set to 1!)
    pinMode(pin, OUTPUT);
    currentState = JT_OFF;
  }
  else {
  // Do not reset outputs on config change (EEPROM re-reads), but update its state
    if (currentState == JT_ON || currentState == JT_OFFDELAY)
      digitalWrite(pin, LOW ^ config->n_inverted);
    else if (currentState == JT_OFF || currentState == JT_ONDELAY)
      digitalWrite(pin, HIGH ^ config->n_inverted);
  }
};


/* standard public function - set a channel, directly or via peering event. Data array contains new value or all peering details */
void HBWSwitchAdvanced::set(HBWDevice* device, uint8_t length, uint8_t const * const data)
{
  hbwdebug(F("set: "));
  if (length == NUM_PEER_PARAMS +2)  // got called with additional peering parameters -- test for correct NUM_PEER_PARAMS
  {
    s_peering_list* peeringList;
    peeringList = (s_peering_list*)data;
    uint8_t currentKeyNum = data[NUM_PEER_PARAMS]; // TODO: add to struct? jtPeeringList? .. seem to use more prog memory...
    bool sameLastSender = data[NUM_PEER_PARAMS +1];

    if ((lastKeyNum == currentKeyNum && sameLastSender) && !peeringList->LongMultiexecute)
    // if ((lastKeyNum == jtPeeringList->currentKeyNum && jtPeeringList->sameLastSender) && !peeringList->LongMultiexecute)
      return;  // repeated long press reveived, but LONG_MULTIEXECUTE not enabled
    // TODO: check behaviour for LONG_MULTIEXECUTE & TOGGLE... (long toggle?)
    
	// hbwdebug(F("\nonDelayTime "));hbwdebug(peeringList->onDelayTime);hbwdebug(F("\n"));
	// hbwdebug(F("onTime "));hbwdebug(peeringList->onTime);hbwdebug(F("\n"));
	// hbwdebug(F("onTimeMin "));hbwdebug(peeringList->isOnTimeMinimal());hbwdebug(F("\n"));
	// hbwdebug(F("offDelayTime "));hbwdebug(peeringList->offDelayTime);hbwdebug(F("\n"));
	// hbwdebug(F("offTime "));hbwdebug(peeringList->offTime);hbwdebug(F("\n"));
	// hbwdebug(F("offTimeMin "));hbwdebug(peeringList->isOffTimeMinimal());hbwdebug(F("\n"));
	// // hbwdebug(F("jtOnDelay "));hbwdebug(peeringList->jtOnDelay);hbwdebug(F("\n"));
	// // hbwdebug(F("jtOn "));hbwdebug(peeringList->jtOn);hbwdebug(F("\n"));
	// // hbwdebug(F("jtOffDelay "));hbwdebug(peeringList->jtOffDelay);hbwdebug(F("\n"));
	// // hbwdebug(F("jtOff "));hbwdebug(peeringList->jtOff);hbwdebug(F("\n"));
	// hbwdebug(F("jtOnDelay "));hbwdebug(jtPeeringList->jtOnDelay);hbwdebug(F("\n"));
	// hbwdebug(F("jtOn "));hbwdebug(jtPeeringList->jtOn);hbwdebug(F("\n"));
	// hbwdebug(F("jtOffDelay "));hbwdebug(jtPeeringList->jtOffDelay);hbwdebug(F("\n"));
	// hbwdebug(F("jtOff "));hbwdebug(jtPeeringList->jtOff);hbwdebug(F("\n"));
    
    if (peeringList->actionType == JUMP_TO_TARGET) {
      hbwdebug(F("jumpToTarget\n"));
      s_jt_peering_list* jtPeeringList;
      jtPeeringList = (s_jt_peering_list*)&data[PEER_PARAM_JT_START];
      jumpToTarget(device, peeringList, jtPeeringList);
    }
    else if (peeringList->actionType >= TOGGLE_TO_COUNTER && peeringList->actionType <= TOGGLE)
    {
      uint8_t nextState;
      switch (peeringList->actionType) {
      case TOGGLE_TO_COUNTER:
        hbwdebug(F("TOGGLE_TO_C\n"));  // switch ON at odd numbers, OFF at even numbers
        // HMW-LC-Sw2 behaviour: toggle actions also use delay and on/off time
        nextState = (currentKeyNum & 0x01) == 0x01 ? JT_ON : JT_OFF;
        break;
      case TOGGLE_INVERS_TO_COUNTER:
        hbwdebug(F("TOGGLE_INV_TO_C\n"));  // switch OFF at odd numbers, ON at even numbers
        nextState = (currentKeyNum & 0x01) == 0x00 ? JT_ON : JT_OFF;
        break;
      case TOGGLE: {
        hbwdebug(F("TOGGLE\n"));
        // toggle to ON when in OFF and ONDELAY state (which would have the output switched OFF)
        nextState = (currentState == JT_OFF || currentState == JT_ONDELAY) ? JT_ON : JT_OFF;
        }
        break;
      default: break;
      }
      uint32_t delay = getDelayForState(nextState, peeringList);  // get on/off time also for toggle actions
      setState(device, nextState, delay, peeringList);
    }
    
    lastKeyNum = currentKeyNum;  // store key press number, to identify repeated key events
  }
  else {
    // set value - no peering event, overwrite any active timer. Peering actions can overwrite INFINITE delay by absolute time_mode only
    // clear stored peeringList on manual SET()
    memset(stateParamList, 0, sizeof(*stateParamList));
    hbwdebug(F("value\n"));
    if (*(data) > 200) {   // toggle
      setState(device, ((currentState == JT_OFF || currentState == JT_ONDELAY) ? JT_ON : JT_OFF), DELAY_INFINITE);
    }
    else {
      setState(device, *(data) == 0 ? JT_OFF : JT_ON, DELAY_INFINITE);  // set on or off
    }
  }
};


/* standard public function - returns length of data array. Data array contains current channel reading */
uint8_t HBWSwitchAdvanced::get(uint8_t* data)
{
  (*data++) = (digitalRead(pin) ^ config->n_inverted) ? 0 : 200;
  *data = stateTimerRunning ? 64 : 0;  // state flag 'working'
  
  return 2;
};


// separate function to set the actual outputs. Would be usually different in derived class
bool HBWSwitchAdvanced::setOutput(HBWDevice* device, uint8_t _newstate)
{
  if (config->output_unlocked)  //0=LOCKED, 1=UNLOCKED
  {
    if (_newstate == JT_ON || _newstate == JT_OFFDELAY) { // JT_OFFDELAY would turn output ON, if not yet ON
      digitalWrite(pin, LOW ^ config->n_inverted);
    }
    else if (_newstate == JT_OFF || _newstate == JT_ONDELAY) { // JT_ONDELAY would turn output OFF, if not yet OFF
      digitalWrite(pin, HIGH ^ config->n_inverted);
    }
    return true;  // return success for unlocked channels, to accept any new state
  }
  return false;
};


void HBWSwitchAdvanced::loop(HBWDevice* device, uint8_t channel)
{
  unsigned long now = millis();

 //*** state machine begin ***//
  if (((now - lastStateChangeTime > stateChangeWaitTime) && stateTimerRunning))
  {
    stopStateChangeTime();  // time was up, so don't come back into here - new valid timer should be set below
    
   #ifdef DEBUG_OUTPUT
     hbwdebug(F("chan:"));
     hbwdebughex(channel);
   #endif
    // check next jump from current state
    uint8_t nextState = getNextState();
    uint32_t delay = getDelayForState(nextState, stateParamList);

    // try to set new state and start new timer, if needed
    setState(device, nextState, delay, stateParamList);
  }
  //*** state machine end ***//
  
  // feedback trigger set?
  checkFeedback(device, channel);
};

