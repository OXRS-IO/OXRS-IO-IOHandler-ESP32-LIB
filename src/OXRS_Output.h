/*
 * OXRS_Output.h
 * 
 * An ESP32 library for handling binary commands and generating
 * control events based on the output configuration. Including 
 * support for interlocks and timers. 
 * 
 */

#ifndef OXRS_OUTPUT_H
#define OXRS_OUTPUT_H

#include "Arduino.h"

// Assume we are dealing with all 16 pins from an MCP23017 
// I2C I/O buffer chip
#define OUTPUT_COUNT                16

// Event constants
#define RELAY_ON                    HIGH
#define RELAY_OFF                   LOW

// Delay between an interlocked deactivation/activation
#define RELAY_INTERLOCK_DELAY_MS    500
#define MOTOR_INTERLOCK_DELAY_MS    2000

// Default timer duration
#define DEFAULT_TIMER_SECS          60

// Output types
enum outputType_t { MOTOR, RELAY, TIMER };

// Special structure to optimise memory usage for storing current state
union outputData_t
{
  uint8_t _data;
  struct 
  {
    uint8_t current : 1;
    uint8_t next : 1;
    uint8_t id : 6;
  } data;
};

// Callback type for onEvent(uint8_t id, uint8_t output, uint8_t type, uint8_t state)
//  * `id` is a custom id (user defined, passed to process()) 
//  * `output` is the output number (0 -> OUTPUT_COUNT - 1)
//  * `type` is RELAY or TIMER
//  * `state` is RELAY_ON or RELAY_OFF
typedef void (*eventCallback)(uint8_t, uint8_t, uint8_t, uint8_t);

class OXRS_Output
{
  public:
    // Initialise the output handler
    void begin(eventCallback, uint8_t defaultType=RELAY);

    // Get/Set the output type
    uint8_t getType(uint8_t output);
    void setType(uint8_t output, uint8_t type);

    // Get/Set interlock linkage
    uint8_t getInterlock(uint8_t output);
    void setInterlock(uint8_t output, uint8_t interlock);

    // Get/Set the timer duration in seconds (for type == TIMER)
    uint16_t getTimer(uint8_t output);
    void setTimer(uint8_t output, uint16_t timer);

    // Call on each MCU loop to keep track of delays and timers
    void process();
    
    // Handle a command to set the state for a specific output
    void handleCommand(uint8_t id, uint8_t output, uint8_t state);

  private:
    // Configuration variables
    uint8_t _type[8];
    uint8_t _interlock[OUTPUT_COUNT];
    uint16_t _timer[OUTPUT_COUNT];

    // State variables
    // _lastUpdateTime: the last time we processed an update, allows for efficient calculation 
    // of event times instead of having to store a full uint32_t for each output (i.e. 16x)
    uint32_t _lastUpdateTime;
    
    // _eventTime[]: incrementing count of how many milliseconds spent in the current state
    uint32_t _eventTime[OUTPUT_COUNT];

    // _delayTime[]: how long an output is being delayed for in milliseconds
    uint32_t _delayTime[OUTPUT_COUNT];

    // _state: structure to store state
    outputData_t _state[OUTPUT_COUNT];

    // Output event callback
    eventCallback _callback;

    // Private methods
    uint8_t _updateOutput(uint8_t id, uint8_t output, uint8_t state);
    void _delayOutput(uint8_t id, uint8_t output, uint8_t state, uint32_t ms);
    uint16_t _getInterlockDelayMs(uint8_t type);
};

#endif