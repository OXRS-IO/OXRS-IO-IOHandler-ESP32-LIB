/*
 * OXRS_Input.h
 * 
 * An ESP32 library capable of detecting input events and reporting
 * consecutive button presses made in quick succession or if the 
 * button was held down for a long time. 
 * 
 */

#ifndef OXRS_INPUT_H
#define OXRS_INPUT_H

#include "Arduino.h"

// DEBOUNCE times (adjust these if you have very noisy buttons or switches)
//  XXX_DEBOUNCE_LOW_MS       debounce delay for the MAKE part of the signal
//  XXX_DEBOUNCE_HIGH_MS      debounce delay for the BREAK part of the signal

// BUTTON types need short debounce times so we don't miss fast multi-click events
#define BUTTON_DEBOUNCE_LOW_MS   15
#define BUTTON_DEBOUNCE_HIGH_MS  30

// ROTARY types need short debounce times so we don't miss rapid rotations
#define ROTARY_DEBOUNCE_LOW_MS   15
#define ROTARY_DEBOUNCE_HIGH_MS  30

// OTHER types can have longer debounce times as we only need to detect simple transitions
#define OTHER_DEBOUNCE_LOW_MS    50
#define OTHER_DEBOUNCE_HIGH_MS   100

// BUTTON types need a few extra times for multi-click and hold event detection
#define BUTTON_MULTI_CLICK_MS    200     // how long to wait for another click before sending a multi-click event
#define BUTTON_HOLD_MS           500     // how long before a click is considered a HOLD event (and repeated)
#define BUTTON_MAX_CLICKS        5       // max count reported in a multi-click event

// Assume we are dealing with a 2 byte IO value - i.e. 16 binary inputs
// typically from an MCP23017 I2C I/O buffer chip
#define INPUT_COUNT              16

// Event constants
// NOTE: 1 to BUTTON_MAX_CLICKS is used to report multi-click events
#define NO_EVENT                 0
#define LOW_EVENT                10
#define HIGH_EVENT               11
// BUTTON events
#define HOLD_EVENT               12
// SECURITY events
#define TAMPER_EVENT             13
#define SHORT_EVENT              14
#define FAULT_EVENT              15

// Rotary encoder state variables
#define ROT_START                0x0
#define ROT_CW_FINAL             0x1
#define ROT_CW_BEGIN             0x2
#define ROT_CW_NEXT              0x3
#define ROT_CCW_BEGIN            0x4
#define ROT_CCW_FINAL            0x5
#define ROT_CCW_NEXT             0x6

// Rotary encoder state table
const unsigned char rotaryState[7][4] = 
{
  // ROT_START
  {ROT_START,     ROT_CW_BEGIN,   ROT_CCW_BEGIN,  ROT_START},
  // ROT_CW_FINAL
  {ROT_CW_NEXT,   ROT_START,      ROT_CW_FINAL,   ROT_START},
  // ROT_CW_BEGIN
  {ROT_CW_NEXT,   ROT_CW_BEGIN,   ROT_START,      ROT_START},
  // ROT_CW_NEXT
  {ROT_CW_NEXT,   ROT_CW_BEGIN,   ROT_CW_FINAL,   ROT_START},
  // ROT_CCW_BEGIN
  {ROT_CCW_NEXT,  ROT_START,      ROT_CCW_BEGIN,  ROT_START},
  // ROT_CCW_FINAL
  {ROT_CCW_NEXT,  ROT_CCW_FINAL,  ROT_START,      ROT_START},
  // ROT_CCW_NEXT
  {ROT_CCW_NEXT,  ROT_CCW_FINAL,  ROT_CCW_BEGIN,  ROT_START},
};

// Rotary encoder event table (which state transitions result in an event)
const unsigned char rotaryEvent[7][4] = 
{
  // ROT_START
  {NO_EVENT,    NO_EVENT,     NO_EVENT,     NO_EVENT},
  // ROT_CW_FINAL
  {NO_EVENT,    NO_EVENT,     NO_EVENT,     LOW_EVENT},
  // ROT_CW_BEGIN
  {NO_EVENT,    NO_EVENT,     NO_EVENT,     NO_EVENT},
  // ROT_CW_NEXT
  {NO_EVENT,    NO_EVENT,     NO_EVENT,     NO_EVENT},
  // ROT_CCW_BEGIN
  {NO_EVENT,    NO_EVENT,     NO_EVENT,     NO_EVENT},
  // ROT_CCW_FINAL
  {NO_EVENT,    NO_EVENT,     NO_EVENT,     HIGH_EVENT},
  // ROT_CCW_NEXT
  {NO_EVENT,    NO_EVENT,     NO_EVENT,     NO_EVENT},
};

// Input types
enum inputType_t { BUTTON, CONTACT, PRESS, ROTARY, SECURITY, SWITCH, TOGGLE };

// Input states
enum inputState_t { IS_HIGH, DEBOUNCE_LOW, IS_LOW, DEBOUNCE_HIGH, AWAIT_MULTI };

// Special structure to optimise memory usage for storing current state and click count
union inputData_t
{
  uint8_t _data;
  struct 
  {
    uint8_t state : 4;
    uint8_t clicks : 4;
  } data;
};

// Callback type for onEvent(uint8_t id, uint8_t button, uint8_t state)
//  * `id` is a custom id (user defined, passed to process()) 
//  * `input` is the input number (0 -> INPUT_COUNT - 1)
//  * `type` is one of BUTTON, CONTACT, PRESS, ROTARY, SECURITY, SWITCH or TOGGLE
//  * `state` is one of;
//    [for BUTTON]
//    - 1, 2, .. MAX_CLICKS   = number of presses (i.e. multi-click)
//    - HOLD_EVENT            = long press (repeats every HOLD_TIME ms)
//    [for CONTACT|SWITCH|TOGGLE]
//    - LOW_EVENT             = HIGH -> LOW transition
//    - HIGH_EVENT            = LOW -> HIGH transition
//    [for PRESS]
//    - LOW_EVENT             = HIGH -> LOW transition
//    [for ROTARY]
//    - LOW_EVENT             = clockwise
//    - HIGH_EVENT            = counter-clockwise
//    [for SECURITY]
//    - HIGH_EVENT            = normal
//    - LOW_EVENT             = alarm
//    - TAMPER_EVENT          = tamper
//    - SHORT_EVENT           = short
//    - FAULT_EVENT           = fault
typedef void (*eventCallback)(uint8_t, uint8_t, uint8_t, uint8_t);

class OXRS_Input
{
  public:
    // Initialise the input handler
    void begin(eventCallback, uint8_t defaultType=SWITCH);

    // Get/Set the input type
    uint8_t getType(uint8_t input);
    void setType(uint8_t input, uint8_t type);

    // Get/Set the invert flag
    uint8_t getInvert(uint8_t input);
    void setInvert(uint8_t input, uint8_t invert);

    // Call on each MCU loop to process input values and raise events
    void process(uint8_t id, uint16_t value);

  private:
    // Configuration variables
    uint8_t _type[8];
    uint16_t _invert;
    
    // Input event callback
    eventCallback _callback;

    // State variables    
    // _lastUpdateTime: the last time we processed an update, allows for efficient calculation 
    // of event times instead of having to store a full uint32_t for each input (i.e. 16x)
    uint32_t _lastUpdateTime;
    
    // _eventTime[]: incrementing count of how many milliseconds spent in the current state
    uint16_t _eventTime[INPUT_COUNT];

    // _state[]: structure to store state and click count in a single byte
    inputData_t _state[INPUT_COUNT];

    // Private methods
    uint8_t _getValue(uint16_t value, uint8_t input);
    uint16_t _getDebounceLowTime(uint8_t type);
    uint16_t _getDebounceHighTime(uint8_t type);
    
    uint8_t _getSecurityState(uint8_t securityValue[]);
    uint8_t _getSecurityEvent(uint8_t securityState);
    
    void _update(uint8_t state[], uint16_t value);
};

#endif
