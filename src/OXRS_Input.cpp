/*
 * OXRS_Input.cpp
 * 
 * An ESP32 library capable of detecting input events and reporting
 * consecutive button presses made in quick succession or if the 
 * button was held down for a long time. 
 *
 */

#include "Arduino.h"
#include "OXRS_Input.h"

void OXRS_Input::begin(eventCallback callback, uint8_t defaultType) 
{
  // Store a reference to our event callback
  _callback = callback; 

  // Initialise our state variables
  _lastUpdateTime = 0;
  for (uint8_t i = 0; i < INPUT_COUNT; i++)
  {
    // Default all inputs
    setType(i, defaultType);
    setInvert(i, 0);

    // Assume all inputs are in-active - i.e. HIGH
    _state[i].data.state = IS_HIGH;
    _state[i].data.clicks = 0;
    
    _eventTime[i] = 0;
  }
}

uint8_t OXRS_Input::getType(uint8_t input)
{
  uint8_t index = input / 2;
  uint8_t bits = (input % 2) * 4;
  
  // shifts the desired 4 bits to the right most position then masks the 4 LSB
  return (_type[index] >> bits) & 0x0F;
}

void OXRS_Input::setType(uint8_t input, uint8_t type)
{
  uint8_t index = input / 2;
  uint8_t bits = (input % 2) * 4;
  
  // sets a mask with the 4 bits we want to change to 0  
  uint8_t mask = ~(0x0F << bits);
  // '& mask' clears, then '| (..)' sets the desired type at desired location 
  _type[index] = (_type[index] & mask) | (type << bits);

  // reset the state for this input ready for processing again
  _state[input].data.state = IS_HIGH;
}

uint8_t OXRS_Input::getInvert(uint8_t input)
{
  // shifts the desired 1 bit to the right most position then masks the LSB
  return (_invert >> input) & 0x01;
}

void OXRS_Input::setInvert(uint8_t input, uint8_t invert)
{
  // sets a mask with the 1 bit we want to change to 0  
  uint16_t mask = ~(0x01 << input);
  // '& mask' clears, then '| (..)' sets the desired type at desired location 
  _invert = (_invert & mask) | ((uint16_t)invert << input);
}

void OXRS_Input::process(uint8_t id, uint16_t value) 
{
  // Process each input to see what, if any, events have occured
  uint8_t event[INPUT_COUNT];
  _update(event, value);

  // Check if we have a callback to handle the press events
  if (_callback) 
  {
    for (uint8_t i = 0; i < INPUT_COUNT; i++)
    {
      // Only interested in inputs with events to report
      if (event[i] != NO_EVENT) 
      {
        _callback(id, i, getType(i), event[i]);
      }
    }
  }
}  

uint8_t OXRS_Input::_getValue(uint16_t value, uint8_t input)
{
  return (bitRead(value, input) ^ getInvert(input));
}

uint16_t OXRS_Input::_getDebounceLowTime(uint8_t type)
{
  switch (type)
  {
    case BUTTON:
      return BUTTON_DEBOUNCE_LOW_MS;
    case ROTARY:
      return ROTARY_DEBOUNCE_LOW_MS;
    default:
      return OTHER_DEBOUNCE_LOW_MS;
  }
}

uint16_t OXRS_Input::_getDebounceHighTime(uint8_t type)
{
  switch (type)
  {
    case BUTTON:
      return BUTTON_DEBOUNCE_HIGH_MS;
    case ROTARY:
      return ROTARY_DEBOUNCE_HIGH_MS;
    default:
      return OTHER_DEBOUNCE_HIGH_MS;
  }
}
  
uint8_t OXRS_Input::_getSecurityState(uint8_t securityValue[])
{
  // Security sensor logic table (using our internal state constants)
  //
  // Sensor     CH1   CH2   CH3   CH4       State           Event
  // -------------------------------------------------------------------
  // NORMAL     OFF   ON    OFF   ON    =>  IS_HIGH         HIGH_EVENT
  // ALARM      OFF   ON    ON    ON    =>  IS_LOW          LOW_EVENT
  // TAMPER     ON    OFF   ON    ON    =>  DEBOUNCE_LOW    TAMPER_EVENT
  // SHORT      OFF   ON    OFF   OFF   =>  DEBOUNCE_HIGH   SHORT_EVENT

  // NORMAL
  if (securityValue[0] == HIGH && securityValue[1] == LOW && securityValue[2] == HIGH && securityValue[3] == LOW)
  {
    return IS_HIGH;
  }

  // ALARM
  if (securityValue[0] == HIGH && securityValue[1] == LOW && securityValue[2] == LOW && securityValue[3] == LOW)
  {
    return IS_LOW;
  }
  
  // TAMPER
  if (securityValue[0] == LOW && securityValue[1] == HIGH && securityValue[2] == LOW && securityValue[3] == LOW)
  {
    return DEBOUNCE_LOW;
  }
  
  // SHORT
  if (securityValue[0] == HIGH && securityValue[1] == LOW && securityValue[2] == HIGH && securityValue[3] == HIGH)
  {
    return DEBOUNCE_HIGH;
  }

  // Ignore any other states
  return AWAIT_MULTI;
}

uint8_t OXRS_Input::_getSecurityEvent(uint8_t securityState)
{
  switch (securityState)
  {
    case IS_HIGH:
      return HIGH_EVENT;
      
    case IS_LOW:
      return LOW_EVENT;
      
    case DEBOUNCE_LOW:
      return TAMPER_EVENT;
      
    case DEBOUNCE_HIGH:
      return SHORT_EVENT;
  }

  return NO_EVENT;
}

void OXRS_Input::_update(uint8_t event[], uint16_t value) 
{
  // Work out how long since our last update so we can increment the event times for each button
  uint16_t delta = millis() - _lastUpdateTime;
  _lastUpdateTime = millis();

  // Read rotary encoder values in pairs (gaps allowed)
  uint8_t rotaryCount = 0;
  uint8_t rotaryValue[2];
  
  // Read security sensor values in quads (a full port)
  uint8_t securityCount = 0;
  uint8_t securityValue[4];
  
  // Process each button (this is not doing any I/O)
  for (uint8_t i = 0; i < INPUT_COUNT; i++)
  {
    // Default to no state - i.e. no event
    event[i] = NO_EVENT;

    // Increment the event time for this button
    _eventTime[i] = _eventTime[i] + delta;

    // Get the configured type of this input
    uint8_t type = getType(i);

    if (type == ROTARY)
    {
      rotaryValue[rotaryCount++] = _getValue(value, i);      

      // Check if we have enough data to determine the rotary event
      if (rotaryCount == 2)
      {        
        // Get the encoder (gray) state, now we have values for both inputs
        unsigned char encoderState = rotaryValue[1] << 1 | rotaryValue[0];

        // Check if this event generates an output (before updating state below)
        event[i] = rotaryEvent[_state[i].data.state][encoderState];

        // Update the state from our state table
        _state[i].data.state = rotaryState[_state[i].data.state][encoderState];

        // Reset for the next rotary encoder
        rotaryCount = 0;
      }
    }
    else if (type == SECURITY)
    {
      securityValue[securityCount++] = _getValue(value, i);      

      if (securityCount == 4)
      {
        uint8_t securityState = _getSecurityState(securityValue);

        // Only generate an event if the state has changed
        if (_state[i].data.state != securityState)
        {
          _state[i].data.state = securityState;
          event[i] = _getSecurityEvent(securityState);
        }
        
        // Reset for the next security sensor
        securityCount = 0;
      }
    }
    else
    {
      // IS_HIGH
      if (_state[i].data.state == IS_HIGH) 
      {
        _state[i].data.clicks = 0;
        if (_getValue(value, i) == LOW) 
        {
          _state[i].data.state = DEBOUNCE_LOW;
          _eventTime[i] = 0;
        }
      } 
      // DEBOUNCE_LOW
      else if (_state[i].data.state == DEBOUNCE_LOW) 
      {
        if (_getValue(value, i) == HIGH)
        {
          // if input bounces before our debounce timer expires then must be a glitch so reset
          _state[i].data.state = IS_HIGH;
          _eventTime[i] = 0;
        }
        else if (_eventTime[i] > _getDebounceLowTime(type)) 
        {
          _state[i].data.state = IS_LOW;
          _eventTime[i] = 0;
  
          // for CONTACT, PRESS, SWITCH or TOGGLE inputs send an event since we have transitioned
          if (type != BUTTON)
          {
            event[i] = LOW_EVENT;
          }
        }  
      } 
      // IS_LOW
      else if (_state[i].data.state == IS_LOW) 
      {
        if (_getValue(value, i) == HIGH) 
        {
          _state[i].data.state = DEBOUNCE_HIGH;
          _eventTime[i] = 0;
        }
        else
        {
          if (type == BUTTON && _eventTime[i] > BUTTON_HOLD_MS) 
          {
            _state[i].data.clicks = HOLD_EVENT;
            _eventTime[i] = 0;
            event[i] = HOLD_EVENT;
          }
        }
      }
      // DEBOUNCE_HIGH
      else if (_state[i].data.state == DEBOUNCE_HIGH) 
      {
        if (_getValue(value, i) == LOW)
        {
          // if input bounces before our debounce timer expires then must be a glitch so reset
          _state[i].data.state = IS_LOW;
          _eventTime[i] = 0;
        }
        else if (_eventTime[i] > _getDebounceHighTime(type)) 
        {
          // for BUTTON inputs check if we have been holding or need to increment the 
          // click count, otherwise for other inputs handle the LOW -> HIGH transition
          if (type != BUTTON)
          {
            _state[i].data.state = IS_HIGH;
            _eventTime[i] = 0;
            
            // only send an event for CONTACT, SWITCH or TOGGLE inputs, for PRESS we are only 
            // interested in HIGH -> LOW transitions so ignore this one
            if (type != PRESS)
            {
              event[i] = HIGH_EVENT;
            }
          }
          else
          {
            if (_state[i].data.clicks == HOLD_EVENT) 
            {
              _state[i].data.state = IS_HIGH;
            } 
            else 
            {
              _state[i].data.clicks = min(BUTTON_MAX_CLICKS, _state[i].data.clicks + 1);
              _state[i].data.state = AWAIT_MULTI;
              _eventTime[i] = 0; 
            }
          }
        }  
      } 
      // AWAIT_MULTI (can only be here for BUTTON inputs)
      else if (_state[i].data.state == AWAIT_MULTI) 
      { 
        if (_getValue(value, i) == LOW) 
        {
          _state[i].data.state = DEBOUNCE_LOW;
          _eventTime[i] = 0;
        } 
        else if (_eventTime[i] > BUTTON_MULTI_CLICK_MS) 
        {
          _state[i].data.state = IS_HIGH;
          event[i] = _state[i].data.clicks;
        } 
      }
    }
  }
}
