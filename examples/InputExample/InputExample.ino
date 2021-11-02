#include <Adafruit_MCP23X17.h>        // For MCP23017 I/O buffers
#include <OXRS_Input.h>               // For input handling

// I/O buffers
Adafruit_MCP23X17 mcp23017;

// Input handlers
OXRS_Input oxrsInput;

void setup() {
  // Initialise serial for debug output
  Serial.begin(115200);

  // Initialise the MCP chip (assume at address 0x20)
  mcp23017.begin_I2C(0x20);

  // Set every pin to be INPUT with internal PULLUPs enabled
  for (uint8_t pin = 0; pin < 16; pin++) {
    mcp23017.pinMode(pin, INPUT_PULLUP);
  }
  
  // Initialise our input handler
  oxrsInput.begin(inputEvent);

  // Set pin 0 to be a BUTTON type and invert
  oxrsInput.setType(0, BUTTON);
  oxrsInput.setInvert(0, 1);
}

void loop() {
  // Read the values for all 16 inputs on this MCP
  uint16_t io_value = mcp23017.readGPIOAB();

  // Check for any input events
  oxrsInput.process(0, io_value);
}

void inputEvent(uint8_t id, uint8_t input, uint8_t type, uint8_t state) {
  char inputType[8];
  getInputType(inputType, type);
  char eventType[7];
  getEventType(eventType, type, state);

  Serial.print(F("[EVENT]"));
  Serial.print(F(" INPUT:"));
  Serial.print(input);
  Serial.print(F(" TYPE:"));
  Serial.print(inputType);
  Serial.print(F(" EVENT:"));
  Serial.println(eventType);
}

void getInputType(char inputType[], uint8_t type)
{
  // Determine what type of input we have
  sprintf_P(inputType, PSTR("ERROR"));
  switch (type)
  {
    case BUTTON:
      sprintf_P(inputType, PSTR("BUTTON"));
      break;
    case CONTACT:
      sprintf_P(inputType, PSTR("CONTACT"));
      break;
    case ROTARY:
      sprintf_P(inputType, PSTR("ROTARY"));
      break;
    case SWITCH:
      sprintf_P(inputType, PSTR("SWITCH"));
      break;
    case TOGGLE:
      sprintf_P(inputType, PSTR("TOGGLE"));
      break;
  }
}

void getEventType(char eventType[], uint8_t type, uint8_t state)
{
  // Determine what event we need to publish
  sprintf_P(eventType, PSTR("ERROR"));
  switch (type)
  {
    case BUTTON:
      switch (state)
      {
        case HOLD_EVENT:
          sprintf_P(eventType, PSTR("HOLD"));
          break;
        case 1:
          sprintf_P(eventType, PSTR("SINGLE"));
          break;
        case 2:
          sprintf_P(eventType, PSTR("DOUBLE"));
          break;
        case 3:
          sprintf_P(eventType, PSTR("TRIPLE"));
          break;
        case 4:
          sprintf_P(eventType, PSTR("QUAD"));
          break;
        case 5:
          sprintf_P(eventType, PSTR("PENTA"));
          break;
      }
      break;
    case CONTACT:
      switch (state)
      {
        case LOW_EVENT:
          sprintf_P(eventType, PSTR("CLOSED"));
          break;
        case HIGH_EVENT:
          sprintf_P(eventType, PSTR("OPEN"));
          break;
      }
      break;
    case ROTARY:
      switch (state)
      {
        case LOW_EVENT:
          sprintf_P(eventType, PSTR("UP"));
          break;
        case HIGH_EVENT:
          sprintf_P(eventType, PSTR("DOWN"));
          break;
      }
      break;
    case SWITCH:
      switch (state)
      {
        case LOW_EVENT:
          sprintf_P(eventType, PSTR("ON"));
          break;
        case HIGH_EVENT:
          sprintf_P(eventType, PSTR("OFF"));
          break;
      }
      break;
    case TOGGLE:
      sprintf_P(eventType, PSTR("TOGGLE"));
      break;
  }
}