/*
 * OXRS_Output.cpp
 * 
 * An ESP32 library for handling binary commands and generating
 * control events based on the output configuration. Including 
 * support for interlocks and timers. 
 *
 */

#include "Arduino.h"
#include "OXRS_Output.h"

void OXRS_Output::begin(eventCallback callback, uint8_t defaultType) 
{
  // Store a reference to our event callback
  _callback = callback; 

  // Initialise our state variables
  for (uint8_t i = 0; i < OUTPUT_COUNT; i++)
  {
    // Default all outputs
    setType(i, defaultType);
    setInterlock(i, i);
    setTimer(i, DEFAULT_TIMER_SECS);

    // Initialise our output state
    _state[i].data.current = RELAY_OFF;
    _state[i].data.next = RELAY_OFF;
    _state[i].data.id = 0;

    _eventTime[i] = 0;
    _delayTime[i] = 0;
  }
}

uint8_t OXRS_Output::getType(uint8_t output)
{
  uint8_t index = output / 2;
  uint8_t bits = (output % 2) * 4;
  
  // shifts the desired 4 bits to the right most position then masks the 4 LSB
  return (_type[index] >> bits) & 0x0F;
}

void OXRS_Output::setType(uint8_t output, uint8_t type)
{
  uint8_t index = output / 2;
  uint8_t bits = (output % 2) * 4;
  
  // sets a mask with the 4 bits we want to change to 0  
  uint8_t mask = ~(0x0F << bits);
  // '& mask' clears, then '| (..)' sets the desired type at desired location 
  _type[index] = (_type[index] & mask) | (type << bits);

  // reset the state for this output ready for processing again
  _eventTime[output] = 0;
  _delayTime[output] = 0;
}

uint8_t OXRS_Output::getInterlock(uint8_t output)
{
  return _interlock[output];
}

void OXRS_Output::setInterlock(uint8_t output, uint8_t interlock)
{
  _interlock[output] = interlock;
}

uint16_t OXRS_Output::getTimer(uint8_t output)
{
  return _timer[output];
}

void OXRS_Output::setTimer(uint8_t output, uint16_t timer)
{
  _timer[output] = timer;
}

void OXRS_Output::process()
{
  // Work out how long since our last update so we can increment the event times for each output
  uint16_t delta = millis() - _lastUpdateTime;
  _lastUpdateTime = millis();

  // Check each output for delay/timer activations
  for (uint8_t i = 0; i < OUTPUT_COUNT; i++)
  {
    // Increment the event time for this output
    _eventTime[i] = _eventTime[i] + delta;

    // Check if this output is waiting for a delay that has expired
    if (_delayTime[i] > 0 && _eventTime[i] > _delayTime[i])
    {
      uint8_t id = _state[i].data.id;  
      uint8_t state = _state[i].data.next;

      _updateOutput(id, i, state);
      _delayTime[i] = 0;
    }
  }
}

void OXRS_Output::handleCommand(uint8_t id, uint8_t output, uint8_t command) 
{
  uint8_t type = getType(output);
  
  if (type == TIMER)
  {
    // Activate/deactivate the output as-per the command
    _updateOutput(id, output, command);

    // If activating then deactivate after timer seconds, otherwise cancel
    if (command == RELAY_ON)
    {
      uint32_t timerMs = (uint32_t)getTimer(output) * 1000;
      _delayOutput(id, output, RELAY_OFF, timerMs);
    }
    else
    {
      _delayTime[output] = 0;
    }
  }
  else
  {
    uint8_t interlock = getInterlock(output);
  
    // Check if output is interlocked and we are activating it
    if (interlock != output && command == RELAY_ON)
    {
      // Deactivate the interlocked output
      if (_updateOutput(id, interlock, RELAY_OFF))
      {
        // Only delay output if our interlock was triggered
        _delayOutput(id, output, RELAY_ON, _getInterlockDelayMs(type));
        return;
      }
    }
  
    // No interlocking so activate the output
    _updateOutput(id, output, command);
  }  
}

uint8_t OXRS_Output::_updateOutput(uint8_t id, uint8_t output, uint8_t state)
{
  // Only do something if the output state has changed
  if (_state[output].data.current == state)
  {
    return 0;
  }

  // Check if we have a callback to handle events
  if (_callback) 
  {
    _callback(id, output, getType(output), state);
  }

  // Update the state of this output
  _state[output].data.current = state;
  return 1;
}

void OXRS_Output::_delayOutput(uint8_t id, uint8_t output, uint8_t state, uint32_t ms)
{
  // Reset the timer for this output and set the delay
  _eventTime[output] = 0;
  _delayTime[output] = ms;

  // Store the next state once the delay expires
  _state[output].data.id = id;  
  _state[output].data.next = state;
}

uint16_t OXRS_Output::_getInterlockDelayMs(uint8_t type)
{
  switch (type)
  {
    case MOTOR:
      return MOTOR_INTERLOCK_DELAY_MS;
    case RELAY:
      return RELAY_INTERLOCK_DELAY_MS;
    default:
      return RELAY_INTERLOCK_DELAY_MS;
  }
}