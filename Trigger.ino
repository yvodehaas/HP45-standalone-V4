/*
  trigger handles the update for all trigger related functions, it updates and states which triggers are active
  These triggers can then be use by other classes and threads to do things
  Trigger was initially housed in position but moved to it's own program because it also needs to trigger buffer.
*/

#include <Arduino.h>

//virtual position values
#define TRIGGER_OFF 0
#define TRIGGER_RISING_EDGE 1
#define TRIGGER_FALLING_EDGE 2
#define TRIGGER_TOGGLE 3
#define TRIGGER_WHILE_HIGH 4
#define TRIGGER_WHILE_LOW 5

//trigger pinmode follows arduino values
#define TRIGGER_FLOATING 0
#define TRIGGER_OUTPUT 1
#define TRIGGER_PULLUP 2
#define TRIGGER_PULLDOWN 3

#define TRIGGER_PINS 9
static uint8_t triggerPin[TRIGGER_PINS] = {24, 25, 26, 33, 34, 0, 1, 38, 37}; //trigger pins are all available broken out pins, in order from used to non-used. Ext pins, encoder pins, serial pins and I2C pins

uint8_t triggerPinMode[TRIGGER_PINS] = {TRIGGER_OFF, TRIGGER_OFF, TRIGGER_OFF, TRIGGER_OFF, TRIGGER_OFF, TRIGGER_OFF, TRIGGER_OFF, TRIGGER_OFF, TRIGGER_OFF}; //what mode each trigger pin has
uint8_t triggerPinResistor[TRIGGER_PINS] = {TRIGGER_FLOATING, TRIGGER_FLOATING, TRIGGER_FLOATING, TRIGGER_PULLUP, TRIGGER_PULLUP, TRIGGER_FLOATING, TRIGGER_FLOATING, TRIGGER_FLOATING, TRIGGER_FLOATING}; //encoder should be pulled up
uint8_t triggerPinHistory[TRIGGER_PINS] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t triggerPushTrigger = 0; //whether or not to push a message if trigger was triggered
uint8_t triggerVirtualFlag = 0; //whether a virtual trigger was called


uint8_t TriggerUpdate() { //updates the trigger and returns a 1 if a new trigger has happened, a 2 if an active trigger is happening (on state change 1 will happen first)
  //look for trigger requirements
  uint8_t temp_active = 0;
  for (uint8_t p = 0; p < TRIGGER_PINS; p++) { //look if any pin is set as a trigger
    if (triggerPinMode[p] != 0) {
      temp_active = 1;
    }
  }
  uint8_t temp_triggered = 0; //if triggered variable

  if (temp_active == 1) { //if any pin is set as a trigger
    uint8_t temp_pin_state[TRIGGER_PINS];
    for (uint8_t p = 0; p < TRIGGER_PINS; p++) { //get pin values
      temp_pin_state[p] = digitalRead(triggerPin[p]);

      //check if any state to trigger is met per pin
      if (triggerPinMode[p] == TRIGGER_RISING_EDGE) { //check rising edge
        if (temp_pin_state[p] == 1 && triggerPinHistory[p] == 0) {
          temp_triggered = 1;
        }
      }
      if (triggerPinMode[p] == TRIGGER_FALLING_EDGE) { //check falling edge
        if (temp_pin_state[p] == 0 && triggerPinHistory[p] == 1) {
          temp_triggered = 1;
        }
      }
      if (triggerPinMode[p] == TRIGGER_TOGGLE) { //check toggle
        if (temp_pin_state[p] != triggerPinHistory[p]) {
          temp_triggered = 1;
        }
      }
      if (triggerPinMode[p] == TRIGGER_WHILE_HIGH) { //check high
        if (temp_pin_state[p] == 1 && triggerPinHistory[p] == 0) {
          temp_triggered = 1;
        }
        else if (temp_pin_state[p] == 1){
          temp_triggered = 2; 
        }
      }
      if (triggerPinMode[p] == TRIGGER_WHILE_LOW) { //check low
        if (temp_pin_state[p] == 0 && triggerPinHistory[p] == 1) {
          temp_triggered = 1;
        }
        else if (temp_pin_state[p] == 0){
          temp_triggered = 2; 
        }
      }

      //set history
      triggerPinHistory[p] = temp_pin_state[p];
    }
  }

  //check virtual triggers
  if (triggerVirtualFlag == 1) {
    triggerVirtualFlag = 0; //reset flag
    temp_triggered = 1;
  }

  if (temp_triggered == 1) { //trigger if required
    if (triggerPushTrigger == 1) { //push trigger message if required
      Serial.println("TRIG");
    }
    return 1;
  }
  else if (temp_triggered == 2){
    return 2;
  }
  return 0;
}

//send a virtual trigger signal that can be passed on in the trigger update function
void TriggerVirtual() {
  triggerVirtualFlag = 1;
}


void TriggerSetPinMode(uint8_t temp_pin, uint8_t temp_mode) {
  if (temp_pin < TRIGGER_PINS) { //limit the input pins
    triggerPinMode[temp_pin] = temp_mode;
  }
}

void TriggerSetResistor(uint8_t temp_pin, uint8_t temp_res) {
  if (temp_pin < TRIGGER_PINS) { //limit the input pins
    if (temp_res == TRIGGER_FLOATING) { //if the pin was set to a mode higher than 0
      pinMode(triggerPin[temp_pin], INPUT); //set the desired pin to input
      triggerPinResistor[temp_pin] = temp_res;
    }
    if (temp_res == TRIGGER_PULLUP) { //if the pin was set to a mode higher than 0
      pinMode(triggerPin[temp_pin], INPUT_PULLUP); //set the desired pin to input
      triggerPinResistor[temp_pin] = temp_res;
    }
    if (temp_res == TRIGGER_PULLDOWN) { //if the pin was set to a mode higher than 0
      pinMode(triggerPin[temp_pin], INPUT_PULLDOWN); //set the desired pin to input
      triggerPinResistor[temp_pin] = temp_res;
    }
  }
}

void TriggerSetTriggerPush(uint8_t temp_mode) {
  temp_mode = constrain(temp_mode, 0, 1);
  triggerPushTrigger = temp_mode;
}
