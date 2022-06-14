/*
   EEPROM handles all storage for all functionality It can save, load and returns all values through functions.
*/

#include <EEPROM.h>

//Print settings (100-198)
#define EEPROM_PRINT_DENSITY 100
#define EEPROM_PRINT_DPI 110
#define EEPROM_PRINT_PRINTHEAD_SIDE 120
uint32_t eepromPrintDensity;
uint32_t eepromPrintDpi;
uint8_t eepromPrintPrintheadSide;

//Print Mode (200-298)
#define EEPROM_PRINT_MODE 200
#define EEPROM_LPI 210
#define EEPROM_VIRTUAL_VELOCITY 220
uint32_t eepromPrintMode;
uint32_t eepromLPI;
uint32_t eepromVirtualVelocity;

//Trigger Mode (300-398)
#define EEPROM_TRIGGER0_MODE 310
#define EEPROM_TRIGGER1_MODE 311
#define EEPROM_TRIGGER2_MODE 312
#define EEPROM_TRIGGER3_MODE 313
#define EEPROM_TRIGGER4_MODE 314
#define EEPROM_TRIGGER5_MODE 315
#define EEPROM_TRIGGER6_MODE 316
#define EEPROM_TRIGGER7_MODE 317
#define EEPROM_TRIGGER8_MODE 318
#define EEPROM_TRIGGER9_MODE 319
uint8_t eepromTriggerMode[10];

#define EEPROM_TRIGGER0_RESISTOR 320
#define EEPROM_TRIGGER1_RESISTOR 321
#define EEPROM_TRIGGER2_RESISTOR 322
#define EEPROM_TRIGGER3_RESISTOR 323
#define EEPROM_TRIGGER4_RESISTOR 324
#define EEPROM_TRIGGER5_RESISTOR 325
#define EEPROM_TRIGGER6_RESISTOR 326
#define EEPROM_TRIGGER7_RESISTOR 327
#define EEPROM_TRIGGER8_RESISTOR 328
#define EEPROM_TRIGGER9_RESISTOR 329
uint8_t eepromTriggerResistor[10];

#define EEPROM_VIRTUAL_TRIGGER_POSITION 320
uint32_t eepromVirtualTriggerPosition;

//buffer modes (400-498)
#define EEPROM_BUFFER_MODE 400
uint8_t eepromBufferMode;

//print text mode (500-598)

//Advanced configurations (1000-1098)
#define EEPROM_PULSE_SPLITS 1000

//Eeprom version number variables (these are the values which need to be in each address to verify that the eeprom has HP45 standalone data in it)
#define EEPROM_CHECK_BYTES 24 
uint16_t eepromCheckByteAddress[EEPROM_CHECK_BYTES] = {0, 99, 199, 299, 399, 499, 599, 699, 799, 899, 999, 1099, 1199, 1299, 1399, 1499, 1599, 1699, 1799, 1899, 1999, 2099, 2199, 2299};
uint8_t eepromCheckByteValues[EEPROM_CHECK_BYTES] = {15, 39, 11, 150, 243, 3, 121, 186, 201, 10, 25, 94, 251, 220, 5, 25, 63, 78, 159, 241, 5, 99, 51, 22};

void EepromLoad() { //Loads all data from EEPROM

}

void EepromSave() { //Saves all data to EEPROM

}

uint8_t EepromCheckByteCollision(uint16_t inputByte) {

  return 0;
}

uint8_t EepromCheckSaved(){ //checks if the current data in Eeprom suggests if HP45 saved to it. Returns a 1 if data was saved to it, and a 0 if no HP45 data was found)
  uint8_t returnValue = 1;
  for (uint8_t e = 0; e < EEPROM_CHECK_BYTES; e++){ //loop through all check bytes
    if (EEPROM.read(eepromCheckByteAddress[e]) != eepromCheckByteValues[e]){
      returnValue = 0;
    }
  }
  return returnValue;
}

void EepromSetSaved(){ //set all eeprom check values to the correct setting
  for (uint8_t e = 0; e < EEPROM_CHECK_BYTES; e++){ //loop through all check bytes
    EEPROM.write(eepromCheckByteAddress[e], eepromCheckByteValues[e]);
  }
}
