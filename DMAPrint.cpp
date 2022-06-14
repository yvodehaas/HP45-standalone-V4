/*  OctoWS2811 - High Performance WS2811 LED Display Library
    http://www.pjrc.com/teensy/td_libs_OctoWS2811.html
    Copyright (c) 2013 Paul Stoffregen, PJRC.COM, LLC
    Some Teensy-LC support contributed by Mark Baysinger.
    https://forum.pjrc.com/threads/40863-Teensy-LC-port-of-OctoWS2811

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

/*
  DMAPrint
  This modification of the OctoWS2811 library is written by Y. de Haas in December 2018

  This modification was made to print inkjet on an HP45 using DMA. It only uses 1 DMA channel
  per port because it prints raw data in parallel.

  Before you go and use this as a reference, remember that I have NO FUCKING CLUE what I am doing
  Use the OctoWS2811 instead: https://www.pjrc.com/teensy/td_libs_OctoWS2811.html

  DMAPrint is used control the HP45 using DMA, with all fast acting stuff on port C and port D.
  All without ever using the processor.

  todo:
  DMA stream still enables all OctoWS stuff
  -in the begin, there are plenty of timers that have no use
  -channel 3 is still kept, though it is not used.
  -on the starting of each dma action, there are timers being set that have no use, and dma3 is still in use.
  -DMA stream is only tested for teensy 3.2 It is rewritten as good as possible for the other option too, but this needs to be tested
  -write read data function. Right now, the data can only be written.
*/

#include <string.h>
#include "DMAPrint.h"

#define CHECK_THRESHOLD 10 //how many short pulses each nozzle receives to test it
#define TEMPERATURE_SENSE_R1 330.0 //the resistance of the temperature sense divider
#define VOLTAGE_SENSE_R1 10000.0 //the resistance of the first resistor in all voltage sensing
#define VOLTAGE_SENSE_R2 1200.0 //the resistance of the second resistor in all voltage sensing
//#define PRINT_DEBUG //uncomment this line to get more in depth reports of all functions over serial

uint16_t DMAPrint::dmaBufferSize;
void * DMAPrint::portCMemory;
void * DMAPrint::portDMemory;
void * DMAPrint::portCWrite;
void * DMAPrint::portDWrite;
uint32_t  DMAPrint::dmaFrequency;
DMAChannel DMAPrint::dma1;
DMAChannel DMAPrint::dma2;
DMAChannel DMAPrint::dma3;

static volatile uint8_t updateInProgress = 0;
static uint32_t update_completed_at = 0;

//pin variables
//pins in use for port C: 9,10,11,12,13,15,22,23
//pins in use for port D: 2,5,6,7,8,14,20,21
//pins for serial: 0,1
static uint8_t primitiveClock = 21; //D6, the clock pin that latches the primitive chanel (also attached to clear)
static uint8_t primtivePins[14] = {15, 22, 23, 9, 10, 13, 11, 12, 2, 14, 7, 8, 6, 20}; //(C0-C7, D0-D5) the array for the primitive pins

static uint8_t addressClock = 5; //D7, the pin that makes the address advance
static uint8_t addressReset = 19; //the reset for the address
static uint8_t headEnable = 3; //the pin that enables the ground of the printhead
static uint8_t nozzleCheck = 4; //the pin that checks the condition of the selected nozzle
static uint8_t senseTSR = A2; //the thermal sense resistor
static uint8_t sense10X = A3; //the 10x calibration resistor

static uint8_t extendedFunctions = 1;
static uint8_t dummy1 = 29; //dummy including address circuitry (no head testing)
static uint8_t dummy2 = 30; //dummy with only the resistor (testing with head in place)
static uint8_t testVoltageLogic = A12; //reference for the logic level line
static uint8_t testVoltageHead = A13; //reference for the head voltage line
static uint8_t testAddress = A17; //test for the address functionality

static uint16_t dpi = 600; //resolution of printhead
static uint16_t dpiRepeat = 1; //how often each to be printed pixel is repeated (calculated from DPI)

//variables
static uint8_t headEnabled; //whether the printhead is enabled or not
static uint8_t pulseSplits = 3; //how many splits there are in a pulse (defaults to 3)
static uint16_t pulseSplit[4][4] = {{16383, 0, 0, 0}, {10922, 5461, 0, 0}, {4681, 9362, 2340, 0}, {8738, 4369, 2184, 1092}}; //bitmask for each of the split pulses for a 1 to 4 way split
static uint16_t burstVar[22]; //a universaly usable variable for a burst
static uint16_t dmaActiveSize; //how much of the actual DMA buffer is used

//nozzle tables
const uint8_t nozzleTableAddress[300] = {
  6, 12, 9, 1, 12, 16, 1, 7, 16, 2,
  7, 13, 2, 19, 13, 5, 19, 18, 5, 3,
  18, 11, 3, 14, 11, 17, 14, 4, 17, 20,
  4, 8, 20, 15, 8, 10, 16, 21, 10, 0,
  21, 6, 0, 17, 6, 12, 9, 1, 12, 16,
  1, 7, 16, 2, 7, 13, 2, 19, 13, 5,
  19, 18, 5, 3, 18, 11, 3, 14, 11, 17,
  14, 4, 17, 20, 4, 8, 20, 15, 8, 10,
  15, 21, 10, 0, 21, 6, 0, 9, 6, 12,
  9, 1, 12, 16, 1, 7, 16, 2, 7, 13,
  2, 19, 13, 5, 19, 18, 5, 3, 18, 11,
  3, 14, 11, 17, 14, 4, 17, 20, 4, 8,
  20, 15, 8, 10, 15, 21, 10, 0, 21, 6,
  0, 9, 6, 12, 9, 1, 12, 16, 1, 7,
  16, 2, 7, 13, 2, 19, 13, 5, 19, 18,
  5, 3, 18, 11, 3, 14, 11, 17, 14, 4,
  17, 20, 4, 8, 20, 15, 8, 10, 15, 21,
  6, 0, 9, 6, 0, 9, 6, 12, 9, 1,
  12, 16, 1, 7, 18, 2, 7, 13, 2, 19,
  13, 5, 19, 18, 5, 3, 18, 11, 3, 14,
  11, 17, 14, 4, 17, 20, 4, 8, 20, 15,
  8, 10, 15, 21, 10, 0, 21, 6, 0, 9,
  6, 12, 9, 1, 12, 16, 1, 7, 16, 2,
  7, 13, 2, 19, 13, 5, 19, 18, 5, 3,
  18, 11, 3, 10, 11, 17, 14, 4, 17, 20,
  4, 8, 20, 15, 8, 10, 15, 21, 10, 0,
  21, 6, 0, 9, 6, 12, 9, 1, 12, 16,
  1, 7, 16, 2, 7, 13, 2, 19, 13, 5,
  19, 18, 5, 3, 18, 11, 3, 14, 11, 17,
  14, 4, 17, 20, 4, 8, 20, 15, 8, 20
};

const uint8_t nozzleTablePrimitive[300] = {
  3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1,
  3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 11, 2, 11, 2, 11, 2, 11, 2, 11, 2, 11, 2, 11, 2, 11, 2, 11, 2, 11, 2,
  11, 2, 11, 2, 11, 2, 11, 2, 11, 2, 11, 2, 11, 2, 11, 2, 11, 2, 11, 2, 11, 2, 11, 2, 10, 8, 10, 8, 10, 8,
  10, 8, 10, 8, 10, 8, 10, 8, 10, 8, 10, 8, 10, 8, 10, 8, 10, 8, 10, 8, 10, 8, 10, 8, 10, 8, 10, 8, 10, 8,
  10, 8, 10, 8, 10, 8, 10, 8, 12, 5, 12, 5, 12, 5, 12, 5, 12, 5, 12, 5, 12, 5, 12, 5, 12, 5, 12, 5, 12, 5,
  12, 5, 12, 5, 12, 5, 12, 5, 12, 5, 12, 5, 12, 5, 12, 5, 12, 5, 12, 5, 12, 5, 7, 9, 7, 9, 7, 9, 7, 9,
  7, 9, 7, 9, 7, 9, 7, 9, 7, 9, 7, 9, 7, 9, 7, 9, 7, 9, 7, 9, 7, 9, 7, 9, 7, 9, 7, 9, 7, 9,
  7, 9, 7, 9, 7, 9, 6, 0, 6, 0, 6, 0, 6, 0, 6, 0, 6, 0, 6, 0, 6, 0, 6, 0, 6, 0, 6, 0, 6, 0,
  6, 0, 6, 0, 6, 0, 6, 0, 6, 0, 6, 0, 6, 0, 6, 0, 6, 0, 6, 0, 4, 13, 4, 13, 4, 13, 4, 13, 4, 13,
  4, 13, 4, 13, 4, 13, 4, 13, 4, 13, 4, 13, 4, 13, 4, 13, 4, 13, 4, 13, 4, 13, 4, 13, 4, 13, 4, 13, 4, 13
};
static int16_t nozzleTableReverse[16][22]; //the return table

DMAPrint::DMAPrint(uint32_t tempBufferSize, void *portCMem , void *portDMem, void *portCWri, void *portDWri, uint32_t tempFrequency)
{
  dmaBufferSize = tempBufferSize;
  portCMemory = portCMem;
  portDMemory = portDMem;
  portCWrite = portCWri;
  portDWrite = portDWri;
  dmaFrequency = tempFrequency;
}

#define WS2811_TIMING_T0H  10 //used to offset trigger D (It wont take same triggers) not all frequencies support all offsets
#define WS2811_TIMING_T1H  176

void DMAPrint::begin(uint32_t tempBufferSize, void *portCMem , void *portDMem, void *portCWri, void *portDWri, uint32_t tempFrequency)
{
  dmaBufferSize = tempBufferSize;
  portCMemory = portCMem;
  portDMemory = portDMem;
  portCWrite = portCWri;
  portDWrite = portDWri;
  dmaFrequency = tempFrequency;
  begin();

  //generate reverse nozzle table ----------------------------------------------------
  for (uint8_t a = 0; a < 22; a++) { //fill -1 in all positions
    for (uint8_t p = 0; p < 16; p++) {
      nozzleTableReverse[p][a] = -1; //set to nothing attached value (-1)
    }
  }
  //delay(2000); Serial.println("Generating reverse table: ");
  for (uint16_t n = 0; n < 300; n++) { //fill the correct nozzle in all filled positions
    nozzleTableReverse[nozzleTablePrimitive[n]][nozzleTableAddress[n]] = n;
    //Serial.print(n); Serial.print(", "); Serial.print(nozzleTablePrimitive[n]); Serial.print(", "); Serial.print(nozzleTableAddress[n]); Serial.println("");
  }
}

//DMA functions ----------------------------------------------------------
void DMAPrint::begin(void) {
  //Serial.println("Starting DMA printhead");
  uint32_t bufsize, frequency;
  bufsize = dmaBufferSize;

  // set up the buffers
  memset(portCMemory, 0, bufsize);
  if (portCWrite) {
    memset(portCWrite, 0, bufsize);
  } else {
    portCWrite = portCMemory;
  }
  memset(portDMemory, 0, bufsize);
  if (portDWrite) {
    memset(portDWrite, 0, bufsize);
  } else {
    portDWrite = portDMemory;
  }

  //declare pins and in-/outputs
  pinMode(primitiveClock, OUTPUT);
  for (uint8_t p = 0; p < 14; p++) {
    pinMode(primtivePins[p], OUTPUT);
  }
  pinMode(addressClock, OUTPUT);
  pinMode(addressReset, OUTPUT);
  pinMode(headEnable, OUTPUT);
  pinMode(nozzleCheck, INPUT);
  pinMode(senseTSR, INPUT);
  pinMode(sense10X, INPUT);

  //extended pin functionality
  pinMode(dummy1, OUTPUT);
  pinMode(dummy2, OUTPUT);
  pinMode(testVoltageLogic, INPUT);
  pinMode(testVoltageHead, INPUT);
  pinMode(testAddress, INPUT);

  // configure the 8 port C output pins
  GPIOC_PCOR = 0xFF;
  pinMode(15, OUTPUT); //C0
  pinMode(22, OUTPUT);  //C1
  pinMode(23, OUTPUT); //C2
  pinMode(9, OUTPUT); //C3
  pinMode(10, OUTPUT); //C4
  pinMode(13, OUTPUT);  //C5
  pinMode(11, OUTPUT);  //C6
  pinMode(12, OUTPUT); //C7

  // configure the 8 port D output pins
  GPIOD_PCOR = 0xFF;
  pinMode(2, OUTPUT); //D0
  pinMode(14, OUTPUT);  //D1
  pinMode(7, OUTPUT); //D2
  pinMode(8, OUTPUT); //D3
  pinMode(6, OUTPUT); //D4
  pinMode(20, OUTPUT);  //D5
  pinMode(21, OUTPUT);  //D6
  pinMode(5, OUTPUT); //D7

  frequency = dmaFrequency;

  //set primitives and address to 0
  //SetPrimitiveClock(1);
  //delayMicroseconds(2);
  //SetPrimitiveClock(0); 
  //AddressReset();

#if defined(__MK20DX128__)
  FTM1_SC = 0;
  FTM1_CNT = 0;
  uint32_t mod = (F_BUS + frequency / 2) / frequency;
  FTM1_MOD = mod - 1;
  FTM1_SC = FTM_SC_CLKS(1) | FTM_SC_PS(0);
  FTM1_C0SC = 0x69;
  FTM1_C1SC = 0x69;
  FTM1_C0V = (mod * WS2811_TIMING_T0H) >> 8;
  FTM1_C1V = (mod * WS2811_TIMING_T1H) >> 8;
  // pin 16 triggers DMA(port B) on rising edge
  CORE_PIN16_CONFIG = PORT_PCR_IRQC(1) | PORT_PCR_MUX(3);
  //CORE_PIN4_CONFIG = PORT_PCR_MUX(3); // testing only*/

#elif defined(__MK20DX256__) //Teensy 3.2 -------------------------------- basic interrupt settings are set here
  FTM2_SC = 0;
  FTM2_CNT = 0;
  uint32_t mod = (F_BUS + frequency / 2) / frequency;
  FTM2_MOD = mod - 1;
  FTM2_SC = FTM_SC_CLKS(1) | FTM_SC_PS(0);
  FTM2_C0SC = 0x69;
  FTM2_C1SC = 0x69;
  FTM2_C0V = (mod * WS2811_TIMING_T0H) >> 8;
  FTM2_C1V = (mod * WS2811_TIMING_T1H) >> 8;
  // pin 32 is FTM2_CH0, PTB18, triggers DMA(port B) on rising edge
  // pin 25 is FTM2_CH1, PTB19
  CORE_PIN32_CONFIG = PORT_PCR_IRQC(1) | PORT_PCR_MUX(3);
  //CORE_PIN25_CONFIG = PORT_PCR_MUX(3); // testing only

#elif defined(__MK64FX512__) || defined(__MK66FX1M0__) //Teensy 3.5 or 3.6 -------------------------------- basic interrupt settings are set here
  FTM2_SC = 0;
  FTM2_CNT = 0;
  uint32_t mod = (F_BUS + frequency / 2) / frequency;
  FTM2_MOD = mod - 1;
  FTM2_SC = FTM_SC_CLKS(1) | FTM_SC_PS(0);
  FTM2_C0SC = 0x69;
  FTM2_C1SC = 0x69;
  FTM2_C0V = (mod * WS2811_TIMING_T0H) >> 8;
  FTM2_C1V = (mod * WS2811_TIMING_T1H) >> 8;
  // FTM2_CH0, PTA10 (not connected), triggers DMA(port A) on rising edge
  PORTA_PCR10 = PORT_PCR_IRQC(1) | PORT_PCR_MUX(3);

#elif defined(__MKL26Z64__)
  FTM2_SC = 0;
  FTM2_CNT = 0;
  uint32_t mod = F_CPU / frequency;
  FTM2_MOD = mod - 1;
  FTM2_SC = FTM_SC_CLKS(1) | FTM_SC_PS(0);
  FTM2_C0SC = FTM_CSC_CHF | FTM_CSC_MSB | FTM_CSC_ELSB;
  FTM2_C1SC = FTM_CSC_CHF | FTM_CSC_MSB | FTM_CSC_ELSB;
  TPM2_C0V = mod - ((mod * WS2811_TIMING_T1H) >> 8);
  TPM2_C1V = mod - ((mod * WS2811_TIMING_T1H) >> 8) + ((mod * WS2811_TIMING_T0H) >> 8);
#endif

  // DMA channel #1 sets WS2811 high at the beginning of each cycle ------------------------------------data to use for each channel is set here
  //write the values from RAM to port D
  dma1.sourceBuffer((uint8_t *)portCMemory, bufsize);
  dma1.destination(GPIOC_PDOR);
  dma1.transferSize(1);
  dma1.transferCount(bufsize);
  dma1.disableOnCompletion();

  //write the values from RAM to port D
  dma2.sourceBuffer((uint8_t *)portDMemory, bufsize);
  dma2.destination(GPIOD_PDOR);
  dma2.transferSize(1);
  dma2.transferCount(bufsize);
  dma2.disableOnCompletion();
  dma2.interruptAtCompletion();

  // DMA channel #3 clear all the pins low at 69% of the cycle

#if defined(__MK20DX128__)
  // route the edge detect interrupts to trigger the 3 channels
  dma1.triggerAtHardwareEvent(DMAMUX_SOURCE_PORTB);
  dma2.triggerAtHardwareEvent(DMAMUX_SOURCE_PORTB);
  DMAPriorityOrder(dma3, dma2, dma1); * /
#elif defined(__MK20DX256__) //<teensy 3.2
  // route the edge detect interrupts to trigger the 3 channels ---------------------------------------------- trigger events set here
  dma1.triggerAtHardwareEvent(DMAMUX_SOURCE_PORTB);
  dma2.triggerAtHardwareEvent(DMAMUX_SOURCE_FTM2_CH0); //triggers on interrupt after port B does, both on portB gave double triggers on dma2
  //dma2.triggerAtHardwareEvent(DMAMUX_SOURCE_FTM2_CH0);
  //dma3.triggerAtHardwareEvent(DMAMUX_SOURCE_FTM2_CH1);
  DMAPriorityOrder(dma3, dma2, dma1);
#elif defined(__MK64FX512__) || defined(__MK66FX1M0__) //<teensy 3.5 or 3.6
  // route the edge detect interrupts to trigger the 3 channels
  dma1.triggerAtHardwareEvent(DMAMUX_SOURCE_PORTA);
  dma2.triggerAtHardwareEvent(DMAMUX_SOURCE_FTM2_CH0); //DMAMUX_SOURCE_PORTA);
  DMAPriorityOrder(dma3, dma2, dma1);
#elif defined(__MKL26Z64__)
  // route the timer interrupts to trigger the 3 channels
  dma1.triggerAtHardwareEvent(DMAMUX_SOURCE_TPM2_CH0);
  dma2.triggerAtHardwareEvent(DMAMUX_SOURCE_TPM2_CH0);
#endif

  // enable a done interrupts when channel #2 completes
  dma2.attachInterrupt(isr); //--------------------------------------------------- Interrupt for completion is set here
  //pinMode(9, OUTPUT); // testing: oscilloscope trigger
}
void DMAPrint::isr(void) {
  //digitalWriteFast(9, HIGH);
  //Serial1.print(".");
  //Serial1.println(dma3.CFG->DCR, HEX);
  //Serial1.print(dma3.CFG->DSR_BCR > 24, HEX);
  dma2.clearInterrupt();
  /*#if defined(__MKL26Z64__)
    GPIOD_PCOR = 0xFF;
    #endif*/
  //Serial1.print("*");
  update_completed_at = micros();
  updateInProgress = 0;
  //digitalWriteFast(9, LOW);
}
int DMAPrint::busy(void) {
  if (updateInProgress) return 1;
  // busy for 50 (or 300 for ws2813) us after the done interrupt, for WS2811 reset
  if (micros() - update_completed_at < 300) return 1;
  return 0;
}

//complex printing functions ----------------------------------------------------------
void DMAPrint::Burst(void) { //<--------------------- change this name to something more representative of DMAPrint
  AddressReset(); //reset the address before printing

  // wait for any prior DMA operation
  //Serial1.print("1");
  while (updateInProgress) ;
  //Serial1.print("2");
  // it's ok to copy the drawing buffer to the frame buffer
  // during the 50us WS2811 reset time
  if (portDWrite != portDMemory) {
    memcpy(portDMemory, portDWrite, dmaBufferSize);
  }
  if (portCWrite != portCMemory) {
    memcpy(portCMemory, portCWrite, dmaBufferSize);
  }
  // wait for WS2811 reset
  while (micros() - update_completed_at < 50) ; //<---------------is this code still required for DMAPrint?
  // ok to start, but we must be very careful to begin
  // without any prior 3 x 800kHz DMA requests pending

#if defined(__MK20DX128__)
  uint32_t cv = FTM1_C0V;
  noInterrupts();
  // CAUTION: this code is timing critical.
  while (FTM1_CNT <= cv) ;
  while (FTM1_CNT > cv) ; // wait for beginning of an 800 kHz cycle
  while (FTM1_CNT < cv) ;
  FTM1_SC = 0;            // stop FTM1 timer (hopefully before it rolls over)
  FTM1_CNT = 0;
  updateInProgress = 1;
  //digitalWriteFast(9, HIGH); // oscilloscope trigger
  PORTB_ISFR = (1 << 0);  // clear any prior rising edge
  uint32_t tmp __attribute__((unused));
  FTM1_C0SC = 0x28;
  tmp = FTM1_C0SC;        // clear any prior timer DMA triggers
  FTM1_C0SC = 0x69;
  FTM1_C1SC = 0x28;
  tmp = FTM1_C1SC;
  FTM1_C1SC = 0x69;
  dma1.enable();
  dma2.enable();          // enable all 3 DMA channels
  dma3.enable();
  FTM1_SC = FTM_SC_CLKS(1) | FTM_SC_PS(0); // restart FTM1 timer
  //digitalWriteFast(9, LOW);

#elif defined(__MK20DX256__) //<teensy 3.2 ------------------------------------------------------------
  //there is a lot of stuff here left from OctoWS, that is not needed for DMAPrint.
  //After the code is tested working, incrementally get rid of this junk
  FTM2_C0SC = 0x28;
  FTM2_C1SC = 0x28;
  uint32_t cv = FTM2_C0V;
  noInterrupts();
  // CAUTION: this code is timing critical.
  while (FTM2_CNT <= cv) ;
  while (FTM2_CNT > cv) ; // wait for beginning of an 800 kHz cycle
  while (FTM2_CNT < cv) ;
  FTM2_SC = 0;             // stop FTM2 timer (hopefully before it rolls over)
  FTM2_CNT = 0;
  updateInProgress = 1;
  //digitalWriteFast(9, HIGH); // oscilloscope trigger
  PORTB_ISFR = (1 << 18);  // clear any prior rising edge
  uint32_t tmp __attribute__((unused));
  FTM2_C0SC = 0x28;
  tmp = FTM2_C0SC;         // clear any prior timer DMA triggers
  FTM2_C0SC = 0x69;
  FTM2_C1SC = 0x28;
  tmp = FTM2_C1SC;
  FTM2_C1SC = 0x69;
  dma1.enable();
  dma2.enable();           // enable all 3 DMA channels
  dma3.enable(); //dma3 no longer used
  FTM2_SC = FTM_SC_CLKS(1) | FTM_SC_PS(0); // restart FTM2 timer
  //digitalWriteFast(9, LOW);

#elif defined(__MK64FX512__) || defined(__MK66FX1M0__) //<Teensy 3.5----------------------------------
  //there is a lot of stuff here left from OctoWS, that is not needed for DMAPrint.
  //After the code is tested working, incrementally get rid of this junk
  FTM2_C0SC = 0x28;
  FTM2_C1SC = 0x28;
  uint32_t cv = FTM2_C1V;
  noInterrupts();
  // CAUTION: this code is timing critical.
  while (FTM2_CNT <= cv) ;
  while (FTM2_CNT > cv) ; // wait for beginning of an 800 kHz cycle
  while (FTM2_CNT < cv) ;
  FTM2_SC = 0;             // stop FTM2 timer (hopefully before it rolls over)
  FTM2_CNT = 0;
  updateInProgress = 1;
  //digitalWriteFast(9, HIGH); // oscilloscope trigger
#if defined(__MK64FX512__)
  asm("nop");
#endif
  PORTA_ISFR = (1 << 10);  // clear any prior rising edge
  uint32_t tmp __attribute__((unused));
  FTM2_C0SC = 0x28;
  tmp = FTM2_C0SC;         // clear any prior timer DMA triggers
  FTM2_C0SC = 0x69;
  FTM2_C1SC = 0x28;
  tmp = FTM2_C1SC;
  FTM2_C1SC = 0x69;
  dma1.enable();
  dma2.enable();           // enable all 3 DMA channels
  dma3.enable();
  FTM2_SC = FTM_SC_CLKS(1) | FTM_SC_PS(0); // restart FTM2 timer
  //digitalWriteFast(9, LOW);

#elif defined(__MKL26Z64__)
  uint32_t sc __attribute__((unused)) = FTM2_SC;
  uint32_t cv = FTM2_C1V;
  noInterrupts();
  while (FTM2_CNT <= cv) ;
  while (FTM2_CNT > cv) ; // wait for beginning of an 800 kHz cycle
  while (FTM2_CNT < cv) ;
  FTM2_SC = 0;    // stop FTM2 timer (hopefully before it rolls over)
  updateInProgress = 1;
  //digitalWriteFast(9, HIGH); // oscilloscope trigger
  dma1.clearComplete();
  dma2.clearComplete();
  dma3.clearComplete();
  uint32_t bufsize = stripLen * 24;
  dma1.transferCount(bufsize);
  dma2.transferCount(bufsize);
  dma3.transferCount(bufsize);
  dma2.sourceBuffer((uint8_t *)frameBuffer, bufsize);
  // clear any pending event flags
  FTM2_SC = FTM_SC_TOF;
  FTM2_C0SC = FTM_CSC_CHF | FTM_CSC_MSB | FTM_CSC_ELSB | FTM_CSC_DMA;
  FTM2_C1SC = FTM_CSC_CHF | FTM_CSC_MSB | FTM_CSC_ELSB | FTM_CSC_DMA;
  // clear any prior pending DMA requests
  dma1.enable();
  dma2.enable();    // enable all 3 DMA channels
  dma3.enable();
  FTM2_CNT = 0; // writing any value resets counter
  FTM2_SC = FTM_SC_DMA | FTM_SC_CLKS(1) | FTM_SC_PS(0);
  //digitalWriteFast(9, LOW);
#endif
  //Serial1.print("3");
  interrupts();
  //Serial1.print("4");
}
void DMAPrint::set(uint32_t tempPosition, uint8_t tempDataC, uint8_t tempDataD) {
  if (tempPosition >= dmaBufferSize) return; //if the write position is higher than possible
  uint8_t *p;
  p = ((uint8_t *) portCWrite) + tempPosition;
  *p = tempDataC;
  p = ((uint8_t *) portDWrite) + tempPosition;
  *p = tempDataD;
}
void DMAPrint::SetBurst(uint16_t temp_input[22], uint8_t temp_mode) { //takes a burst array and writes it to the DMA buffer (mode is long or short pulses. 1 is long, 0 is short)
  //the DMA needs to be filled acording to a certain pattern. This pattern make the printhead fire properly.
  //C0-C7 and D0-D5 are the primitive select pins. D6 is primitive clock (and clear, they are connected). D7 is address next.
  //The signal goes as follows in steps of 0.9us (All unmentioned keep state):
  //1:Address high
  //2:address low
  //3:first 1/3rd of primitive pins change state (high or low depending on the input), clock pin high
  //4:*idle
  //5:all primitives low, clock pin low
  //6:second 1/3rd of primitive pins change state (high or low depending on the input), clock pin high
  //7:*idle
  //8:all primitives low, clock pin low
  //9:third 1/3rd of primitive pins change state (high or low depending on the input), clock pin high
  //10:*idle
  //11:all primitives low, clock pin low
  //go to 1 and repeat for the next address (and data in input)
  //(*all idles are optional based on the mode. In long mode they are added, in short mode these are not added to the buffer)

  //(edit: changed for testing. First data, then clock, not at the same time)

  //make standard values:
  uint8_t tempAllOff[] = {0, 0};
  uint8_t tempAddressNext[] = {0, 0B10000000};
  uint8_t tempPrimitive_high[] = {0, 0B01000000};

  dmaActiveSize = 0; //set active size to 0
  uint8_t tempPulse[2]; //make pulse to print variable
  uint16_t tempPulseUnsplit;

  for (uint8_t a = 0; a < 22; a++) { //fill in data for all addresses
    set(dmaActiveSize, tempAddressNext[0], tempAddressNext[1]); //make address high
    dmaActiveSize ++;
    set(dmaActiveSize, tempAllOff[0], tempAllOff[1]); //make address low
    dmaActiveSize ++;
    for (uint8_t p = 0; p < pulseSplits; p++) {
      tempPulseUnsplit = temp_input[a] & pulseSplit[pulseSplits - 1][p]; //overlay bitmask for the splits
      //Serial.print("Data on "); Serial.print(p); Serial.print(", number: "); Serial.println(tempPulseUnsplit);
      //Serial.print("filter: "); Serial.println(pulseSplit[p]);
      tempPulse[0] = tempPulseUnsplit & 255; //set port C
      //tempPulse[0] |= tempPrimitive_high[0]; //add primtive clock
      tempPulse[1] = (tempPulseUnsplit >> 8) & 255;  //set port D
      //tempPulse[1] |= tempPrimitive_high[1]; //add primtive clock

      set(dmaActiveSize, tempPulse[0], tempPulse[1]); //clock in p'th third
      dmaActiveSize ++;

      //set clocks
      tempPulse[0] |= tempPrimitive_high[0]; //add primtive clock
      tempPulse[1] |= tempPrimitive_high[1]; //add primtive clock
      set(dmaActiveSize, tempPulse[0], tempPulse[1]); //clock in p'th third
      dmaActiveSize ++;

      if (temp_mode == 1) {
        set(dmaActiveSize, tempPulse[0], tempPulse[1]); //optional idle
        dmaActiveSize ++;
      }
      set(dmaActiveSize, tempAllOff[0], tempAllOff[1]); //all off
      dmaActiveSize ++;
    }
  }
  //backfill remainder with zeros
  for (uint32_t b = dmaActiveSize; b < dmaBufferSize; b++) {
    set(dmaActiveSize, tempAllOff[0], tempAllOff[1]); //all off
    dmaActiveSize++;
  }
}
//takes an empty uint8_t array of 300 as an input for nozzles and returns the state of each nozzle (0 for broken, 1 for working)
//takes an empty uint8_t array of 22 as input for addresses and returns the number of working nozzles on each address
//takes an empty uint8_t array of 14 as input for the primitives, and returns the number of working nozzles on each primitive
void DMAPrint::TestHead(uint8_t* tempNozzleState, uint8_t* tempAddressState, uint8_t* tempPrimitiveState) { 
  int16_t testPulse;
  uint8_t tempAddress, tempPrimitive;
  int8_t tempTests;
  uint8_t headPresent = 0; //if the printhead is attached or not

  for (uint8_t a = 0; a < 22; a++){ //reset addresses
    tempAddressState[a] = 0;
  }
  for (uint8_t p = 0; p < 14; p++){ //reset primitive
    tempPrimitiveState[p] = 0;
  }

  for (uint16_t n = 0; n < 300; n++) {
    tempAddress = nozzleTableAddress[n];
    tempPrimitive = nozzleTablePrimitive[n];

    //go to address
    AddressReset();
    AddressNext();
    for (uint8_t a = 0; a < tempAddress; a++) { //step to the right address
      AddressNext();
    }
    testPulse = 0;
    bitWrite(testPulse, tempPrimitive, 1); //set right primitive in pulse

    //discharge the capacitor
    SetEnable(1);
    delayMicroseconds(50);
    SetEnable(0);

    tempTests = CHECK_THRESHOLD;

    //test nozzle
    while (1) {
      //do a pulse
      PrimitiveShortPulse(testPulse);

      //check if test pin is low (test circuit pulls down on positive)
      if (GetNozzleCheck() == 0) {
        break;
      }
      tempTests--; //subtract one from tests
      if (tempTests <= 0) { //if the max number of tests is reached
        break;
      }
    }

    if (tempTests == 0) { //if variable reached 0, nozzle never tested positive
      tempNozzleState[n] = 0;
    }
    else { //if anything other than 0, nozzle is positive
      tempNozzleState[n] = 1;
      tempAddressState[tempAddress] ++; //add one to address
      tempPrimitiveState[tempPrimitive] ++; //add one to primitive
    }
  }
}
uint8_t DMAPrint::TestDummy(uint8_t tempDummy) { //test the given dummy nozzle
  int8_t tempTests;

  //discharge the capacitor
  SetEnable(1);
  delayMicroseconds(50);
  SetEnable(0);

  tempTests = CHECK_THRESHOLD;

  //test nozzle
  while (1) {
    //do a pulse
    PrimitiveDummyPulse(tempDummy);

    //check if test pin is low (test circuit pulls down on positive)
    if (GetNozzleCheck() == 0) {
      break;
    }
    tempTests--; //subtract one from tests
    if (tempTests <= 0) { //if the max number of tests is reached
      break;
    }
  }

  if (tempTests == 0) { //if variable reached 0, nozzle never tested positive
    return 0;
  }

  //test if enable pulls down
  //Serial.println(GetNozzleCheck());
  SetEnable(1);
  delayMicroseconds(50);
  SetEnable(0);
  //Serial.println(GetNozzleCheck());
  if (GetNozzleCheck() == 0) { //if capacitor was not discharged, enable is broken, return not falling
    return 2; 
  }
  return 1;
}
void DMAPrint::SingleNozzle(uint16_t tempNozzle) { //triggers a single indicated nozzle
  tempNozzle = constrain(tempNozzle, 0, 299);
  ResetBurst(); //reset the burst variable
  bitWrite(burstVar[nozzleTableAddress[tempNozzle]], nozzleTablePrimitive[tempNozzle], 1); //get the right nozzle to fire
  SetBurst(burstVar, 1); //set the burst as a long pulses
  Burst();
}
int8_t DMAPrint::Preheat(uint16_t tempPulses) { //does a given number of short pulses on the printhead to preheat the nozzles
  uint16_t tempPulse = 16383;
  //Serial.print("Preheating: "); Serial.println(tempPulses);
  if (headEnabled == 0) return 0; //check if burst is possible, return a 0 if not
  //Serial.print("Head enabled, preheating");
  //tempPulses = constrain(tempPulses, 0, maxPreheatPulses);
  for (uint8_t a = 0; a < 22; a++) {
    burstVar[a] = tempPulse;
  }
  SetBurst(burstVar, 0); //set the burst as a short pulses
  for (uint16_t pulses = 0; pulses < tempPulses; pulses++) {
    Burst();
  }
  return 1; //return a 1 if successful
}
int8_t DMAPrint::Prime(uint16_t tempPulses) { //does a given number of long pulses on the printhead to start the nozzles
  uint16_t tempPulse = 16383;
  //Serial.print("Preheating: "); Serial.println(tempPulses);
  if (headEnabled == 0) return 0; //check if burst is possible, return a 0 if not
  //Serial.print("Head enabled, preheating");
  //tempPulses = constrain(tempPulses, 0, maxPreheatPulses);
  for (uint8_t a = 0; a < 22; a++) {
    burstVar[a] = tempPulse;
  }
  SetBurst(burstVar, 1); //set the burst as a long pulses
  for (uint16_t pulses = 0; pulses < tempPulses; pulses++) {
    Burst();
  }
  return 1; //return a 1 if successful
}
uint8_t DMAPrint::TestAddress(void) { //tests if the address circuit fully cycles, 1 for functional, 0 for not
#ifdef PRINT_DEBUG
  Serial.println("Testing address functionality");
#endif
  uint8_t tempReturn = 1;
  AddressReset(); //reset address
  delay(1);
  uint32_t maxLowVoltage = 2000; //the highest accepted voltage for a low level
  uint32_t minHighVoltage = 10000; //the lowest accepted voltage for a high level
  uint32_t maxHighVoltage = 14000; //the highest accepted voltage for a high level
  uint32_t tempMeasurement;

  //loop through all addresses
  for (uint8_t a = 0; a < 23; a++) { //loop through 0 position and 22 addresses
    //test if the check pin is low
    tempMeasurement = GetVoltageAddress();
#ifdef PRINT_DEBUG
    Serial.println(tempMeasurement);
#endif
    //if the address is not low, set flag
    if (tempMeasurement > maxLowVoltage) {
      tempReturn = 0;
#ifdef PRINT_DEBUG
      Serial.println("Unwanted high in any address");
#endif
    }
    //go to the next address
    AddressNext();
    delay(1);
  }

  //check overflow of the address
  tempMeasurement = GetVoltageAddress();
#ifdef PRINT_DEBUG
  Serial.println(tempMeasurement);
#endif
  if (tempMeasurement < minHighVoltage || tempMeasurement > maxHighVoltage) { //if voltage is out of range
    tempReturn = 0;
#ifdef PRINT_DEBUG
    Serial.println("Expected address not high");
#endif
  }

  AddressReset(); //reset

  //return state
  return tempReturn;
}
//calibrate (future)


//simple pin triggers ----------------------------------------------------------
void DMAPrint::PrimitivePulse(uint16_t tempState) { //do one pulse of the printhead with the specified input on the primitives (LSB to MSB, P0 to P13)
  SetPrimitiveClock(0); //set clock to 0
  SetPrimitivePins(tempState); //set primitive pins
  noInterrupts(); //allow no interrupts during pulse
  SetPrimitiveClock(1); //set clock to 1
  for (uint8_t d = 0; d < 12; d++) { //NOP delay (12 loops of 8 NOP's is ca 1.8us) (10=1.6, 14=2.1, 17=2.4)
    __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"); //8 NOP's
  }
  SetPrimitiveClock(0); //set clock to 0
  interrupts(); //allow interrupts again
}
void DMAPrint::PrimitiveShortPulse(uint16_t tempState) { //do one pulse of the printhead with the specified input on the primitives for a shorter while (LSB to MSB, P0 to P13)
  SetPrimitiveClock(0); //set clock to 0
  SetPrimitivePins(tempState); //set primitive pins
  noInterrupts(); //allow no interrupts during pulse
  SetPrimitiveClock(1); //set clock to 1
  __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"); //18 NOP's, ca. 200ns
  __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"); //18 NOP's, ca. 200ns
  __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"); //18 NOP's, ca. 200ns
  SetPrimitiveClock(0); //set clock to 0
  interrupts(); //allow interrupts again
}
void DMAPrint::PrimitiveDummyPulse(uint8_t tempDummy) { //do one pulse of the printhead with the specified input on the primitives for a shorter while (LSB to MSB, P0 to P13)
  SetPrimitiveClock(0); //set clock to 0
  if (tempDummy == 0) {
    digitalWrite(dummy1, 1); //pulldown dummy
  }
  else if (tempDummy == 1) {
  digitalWrite(dummy2, 1); //ungrounded dummy
  }

  noInterrupts(); //allow no interrupts during pulse
  SetPrimitiveClock(1); //set clock to 1
  __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"); //18 NOP's, ca. 200ns
  __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"); //18 NOP's, ca. 200ns
  __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"); //18 NOP's, ca. 200ns
  SetPrimitiveClock(0); //set clock to 0
  interrupts(); //allow interrupts again
  digitalWrite(dummy1, 0);
  digitalWrite(dummy2, 0);
}
void DMAPrint::AddressNext(void) { //reset clock
  SetAddressClock(1);
  delayMicroseconds(1);
  SetAddressClock(0);
  delayMicroseconds(1);
}
void DMAPrint::AddressReset(void) { //reset address
  SetAddressReset(1);
  delayMicroseconds(1);
  SetAddressReset(0);
  delayMicroseconds(1);
}

//Read values ----------------------------------------------------------
uint32_t DMAPrint::GetTSRRaw(uint8_t tempResolution) { //read TSR
  tempResolution = constrain(tempResolution, 0, 13); //limit resolution input
  analogReadResolution(tempResolution); //set resolution
  uint16_t temp = analogRead(senseTSR);
  analogReadResolution(10); //return resolution to 10 (default)
  return temp;
}
uint32_t DMAPrint::Get10XRaw(uint8_t tempResolution) { //Read 10x
  tempResolution = constrain(tempResolution, 0, 13); //limit resolution input
  analogReadResolution(tempResolution); //set resolution
  uint16_t temp = analogRead(sense10X);
  analogReadResolution(10); //return resolution to 10 (default)
  return temp;
}
uint32_t DMAPrint::GetVoltageLogicRaw(uint8_t tempResolution) { //read address logic voltage
  tempResolution = constrain(tempResolution, 0, 13); //limit resolution input
  analogReadResolution(tempResolution); //set resolution
  uint16_t res = analogRead(testVoltageLogic);
  analogReadResolution(10); //return resolution to 10 (default)
  return res;
}
uint32_t DMAPrint::GetVoltageHeadRaw(uint8_t tempResolution) { //read primitive drive voltage
  tempResolution = constrain(tempResolution, 0, 13); //limit resolution input
  analogReadResolution(tempResolution); //set resolution
  uint16_t res = analogRead(testVoltageHead);
  analogReadResolution(10); //return resolution to 10 (default)
  return res;
}
uint32_t DMAPrint::GetVoltageAddressRaw(uint8_t tempResolution) { //read primitive drive voltage
  tempResolution = constrain(tempResolution, 0, 13); //limit resolution input
  analogReadResolution(tempResolution); //set resolution
  uint16_t res = analogRead(testAddress);
  analogReadResolution(10); //return resolution to 10 (default)
  return res;
}
uint8_t DMAPrint::GetNozzleCheck(void) { //Read nozzle check
  return digitalRead(nozzleCheck);
}
int32_t DMAPrint::GetTemperature(void) { //enables the head if required and calculates temperature (in .1C. 200 is 20.0C)
  SetEnableTemp(1); //set the head (temporarily) to enabled while checking
  AddressReset(); //reset address because else for magical reasons the 10X will fail to read

  int16_t temp10x = Get10XRaw(13); //get analog read in 13 bit resolution
  int16_t tempTsr = GetTSRRaw(13); //get analog read in 13 bit resolution
  const float tempVin = 3.3;
  const float tempR1 = TEMPERATURE_SENSE_R1;

  //Vout = (R2 / (R2 + R1)) * Vin, R1 is 330ohm
  //R2 = ((Vout x R1)/(Vin-Vout))

  //calculate in celcius
  float tempFCalc;
  //get the 10x voltage
  tempFCalc = float(temp10x);
  tempFCalc /= 8192; //get the fraction
  tempFCalc *= tempVin; //get the voltage
  float temp10xRes = ((tempFCalc * tempR1) / (tempVin - tempFCalc)); //get the 10x resistance
  //Serial.print("10X resistor: "); Serial.println(temp10xRes);

  //get the tsr voltage
  tempFCalc = float(tempTsr);
  tempFCalc /= 8192; //get the fraction
  tempFCalc *= tempVin; //get the voltage
  float tempTsrRes = ((tempFCalc * tempR1) / (tempVin - tempFCalc)); //get the tsr resistance
  //Serial.print("TSR resistor: "); Serial.println(tempTsrRes);

  //check both to be reasonable values
  if (temp10xRes < 150.0 || temp10xRes > 500.0) return -2;
  if (tempTsrRes < 150.0 || tempTsrRes > 500.0) return -2;

  //get the TSR - 10X
  //at 10 ohms, the temperature is 20C, for every 1.1 ohms the temperature rises 1 degree
  //T = 1.1R+10
  tempFCalc = tempTsrRes - temp10xRes;
  float tempTemp = 1.1 * tempFCalc + 10.0;
  tempTemp *= 10.0; //multiply by ten to add the decimal value
  //Serial.print("Temperature: "); Serial.println(tempTemp);
  return long(tempTemp); //return celcius

  EnableReset(); //set the head to previous state
}
uint32_t DMAPrint::GetVoltageLogic(void) { //reads the voltage of the printhead logic and returns it in millivolts
  int16_t tempV = GetVoltageLogicRaw(13); //get analog read in 13 bit resolution
  //Vout = (R2 / (R2 + R1)) * Vin
  //Vin = ((R2+R1) / R2) * Vout <this one is used
  float tempFCalc = float(tempV); //write value to the float variable
  tempFCalc /= 8192.0; //get the fraction
  tempFCalc *= 3.3; //create Vout value
  float tempFCalc2 = VOLTAGE_SENSE_R1 + VOLTAGE_SENSE_R2;
  tempFCalc2 /= VOLTAGE_SENSE_R2;
  tempFCalc = tempFCalc2 * tempFCalc; //make output voltage
  tempFCalc *= 1000.0;
  return long(tempFCalc);
}
uint32_t DMAPrint::GetVoltageHead(void) { //reads the voltage of the printhead driving circuitry
  int16_t tempV = GetVoltageHeadRaw(13); //get analog read in 13 bit resolution
  //Vout = (R2 / (R2 + R1)) * Vin
  //Vin = ((R2+R1) / R2) * Vout <this one is used
  float tempFCalc = float(tempV); //write value to the float variable
  tempFCalc /= 8192.0; //get the fraction
  tempFCalc *= 3.3; //create Vout value
  float tempFCalc2 = VOLTAGE_SENSE_R1 + VOLTAGE_SENSE_R2;
  tempFCalc2 /= VOLTAGE_SENSE_R2;
  tempFCalc = tempFCalc2 * tempFCalc; //make output voltage
  tempFCalc *= 1000.0;
  return long(tempFCalc);
}
uint32_t DMAPrint::GetVoltageAddress(void) { //reads the voltage of the printhead driving circuitry
  int16_t tempV = GetVoltageAddressRaw(13); //get analog read in 13 bit resolution
  //Vout = (R2 / (R2 + R1)) * Vin
  //Vin = ((R2+R1) / R2) * Vout <this one is used
  float tempFCalc = float(tempV); //write value to the float variable
  tempFCalc /= 8192.0; //get the fraction
  tempFCalc *= 3.3; //create Vout value
  float tempFCalc2 = VOLTAGE_SENSE_R1 + VOLTAGE_SENSE_R2;
  tempFCalc2 /= VOLTAGE_SENSE_R2;
  tempFCalc = tempFCalc2 * tempFCalc; //make output voltage
  tempFCalc *= 1000.0;
  return long(tempFCalc);
}

//Raw pin modifications ----------------------------------------------------------
void DMAPrint::SetPrimitiveClock(uint8_t tempState) { //set primitive clock
  if (tempState == 1) {
    digitalWrite(primitiveClock, 1);
  }
  else {
    digitalWrite(primitiveClock, 0);
  }
}
void DMAPrint::SetPrimitivePins(uint16_t tempState) { //set primitive pins
  uint8_t tempValue;
  for (uint8_t p = 0; p < 14; p++) { //loop through
    tempValue = bitRead(tempState, p); //get correct bit from value
    digitalWrite(primtivePins[p], tempValue); //
  }
}
void DMAPrint::SetAddressClock(uint8_t tempState) { //set address next
  if (tempState == 1) {
    digitalWrite(addressClock, 1);
  }
  else {
    digitalWrite(addressClock, 0);
  }
}
void DMAPrint::SetAddressReset(uint8_t tempState) { //set address reset
  if (tempState == 1) {
    digitalWrite(addressReset, 1);
  }
  else {
    digitalWrite(addressReset, 0);
  }
}
void DMAPrint::SetEnable(uint8_t tempState) { //sets the enable state of the printhead to 0 or 1
  if (tempState == 1) {
    digitalWrite(headEnable, 1);
    headEnabled = 1; //set enabled state to 1
  }
  else {
    digitalWrite(headEnable, 0);
    headEnabled = 0; //set enabled state to 0
  }
}
void DMAPrint::SetEnableTemp(uint8_t tempState) { //sets the enable state of the printhead to 0 or 1, but does not alter the enabled variable
  if (tempState == 1) {
    digitalWrite(headEnable, 1);
  }
  else {
    digitalWrite(headEnable, 0);
  }
}
void DMAPrint::EnableReset() { //set enable to last official state
  digitalWrite(headEnable, headEnabled);
}

//Variable functions ----------------------------------------------------------
void DMAPrint::ResetBurst(void) { //resets the burst variable
  for (uint8_t a = 0; a < 22; a++) {
    burstVar[a] = 0;
  }
}
uint8_t DMAPrint::GetPrimitive(uint16_t tempNozzle) { //returns the primitive location of a given nozzle
  tempNozzle = constrain(tempNozzle, 0, 299);
  return nozzleTablePrimitive[tempNozzle];
}
uint8_t DMAPrint::GetAddress(uint16_t tempNozzle) { //returns the address location of a given nozzle
  tempNozzle = constrain(tempNozzle, 0, 299);
  return nozzleTableAddress[tempNozzle];
}
int16_t DMAPrint::GetNozzle(uint8_t tempPrimitive, uint8_t tempAddress) { //returns the nozzle of a primitive and address, -1 if no nozzle is there
  tempPrimitive = constrain(tempPrimitive, 0, 13);
  tempAddress = constrain(tempAddress, 0, 21);
  return nozzleTableReverse[tempPrimitive][tempAddress];
}
uint16_t *DMAPrint::ConvertB6RawToBurst(uint8_t temp_input[50], uint16_t temp_burst[22]) { //takes an array of 50 bytes where the 6 LSB are nozzle on or off, starting at 0 and ending at 299 and converts to a pointed uint16_t[22] burst array
  uint16_t tempNozzle = 0; //keeps track of the current nozzle
  uint8_t temp_state; //used to write on or off to
  uint8_t temp_add, temp_prim;
  for (uint8_t a = 0; a < 22; a++) {
    temp_burst[a] = 0;
  }
  for (uint8_t B = 0; B < 50; B++) { //bytes within byte
    for (uint8_t b = 0; b < 6; b++) { //bits within byte
      for (uint8_t r = 0; r < dpiRepeat; r++) { //repeat pixels
        if (tempNozzle < 300) { //only look if nozzle is lower than 300 (some resolutions give halve filled B64 values)
          temp_add = nozzleTableAddress[tempNozzle];
          temp_prim = nozzleTablePrimitive[tempNozzle];
          //if (b%2 == 1){ //TEMPORARY, ONLY PRINT ODD OR EVEN
          temp_state = bitRead(temp_input[B], b); //get on or off from input
          //}
          //else {
          //  temp_state = 0;
          //}
          bitWrite(temp_burst[temp_add], temp_prim, temp_state); //set nozzle in burst on or off
          tempNozzle ++; //add one to nozzle
        }
      }
    }
  }
  return temp_burst;
}
uint16_t *DMAPrint::ConvertB6ToggleToBurst(uint8_t temp_input[50], uint16_t temp_burst[22]) { //takes raw data in toggle format and converts it to burst
  uint16_t tempNozzle = 0; //keeps track of the current nozzle
  uint8_t tempState = 1; //used to write on or off to
  uint8_t tempValue;
  uint8_t temp_add, temp_prim;
  for (uint8_t B = 0; B < 50; B++) { //loop through all array values, turn on and off, stop when 300 is reached
    tempValue = temp_input[B];
    for (uint8_t R = 0; R < tempValue; R++) { //for the amount of repeats given
      for (uint8_t r = 0; r < dpiRepeat; r++) { //repeat pixels
        temp_add = nozzleTableAddress[tempNozzle];
        temp_prim = nozzleTablePrimitive[tempNozzle];
        bitWrite(temp_burst[temp_add], temp_prim, tempState); //set nozzle in burst on or off
        tempNozzle++; //go to next nozzle
        if (tempNozzle == 300) { //if 300 is reached, end
          return temp_burst; //return value if 300 is reached
        }
      }
    }
    //toggle state
    if (tempState == 1) tempState = 0;
    else tempState = 1;
  }
  return temp_burst;
}
uint16_t *DMAPrint::ConvertB8ToBurst(uint8_t temp_input[38], uint16_t temp_burst[22]) { //takes an array of 38 bytes where the 8 LSB are nozzle on or off, starting at 0 and ending at 299 and converts to a pointed uint16_t[22] burst array
  uint16_t tempNozzle = 0; //keeps track of the current nozzle
  uint8_t temp_state; //used to write on or off to
  for (uint8_t B = 0; B < 38; B++) { //bytes within byte
    for (uint8_t b = 0; b < 8; b++) { //bits within byte
      for (uint8_t r = 0; r < dpiRepeat; r++) { //repeat pixels
        temp_state = bitRead(temp_input[B], b); //get on or off from input
        bitWrite(temp_burst[nozzleTableAddress[tempNozzle]], nozzleTablePrimitive[tempNozzle], temp_state); //set nozzle in burst on or off
        tempNozzle ++; //add one to nozzle
      }
    }
  }
  return temp_burst;
}
void DMAPrint::SetDPI(uint16_t temp_dpi) { //takes dpi, does some maths on it and sets the resolution on decoding
  if (temp_dpi <= 600) { //on valid dip (max is 600
    dpiRepeat = 600 / temp_dpi; //calculate repeat value. Repeat is how often each pixel is repeated going from nozzle 0 to 299
    dpi = 600 / dpiRepeat; //get actual DPI, based on input, rounding to nearest number that is a division from 600
    //Serial.print("Setting DPI to: "); Serial.println(dpi);
  }
}

void DMAPrint::DMASetPulseSplit(uint8_t tempSplit){ //sets the number of divisions (splits) in each pulse, 1 being no splits and 4 being 4 powerings per pulse
  tempSplit = constrain(tempSplit, 1, 4);
  pulseSplits = tempSplit;
}

uint8_t DMAPrint::DMAGetPulseSplit(){ //returns the number of splits currently being used
  return pulseSplits;
}

uint8_t DMAPrint::GetEnabledState() { //returns the current head enable state
  return headEnabled;
}
//write pin raw
uint8_t DMAPrint::WritePinRaw(uint32_t temp_input) {
  //takes the input that was converted from text to number, and writes it raw.
  //Command is the 3 character designation on the PCB, and a 1 or 0. This function will decode it.
  //Serial.println("Writing to pin raw (No protections!)");
  char temp_pin[3];
  uint32_t temp_pin_number;
  uint8_t temp_pin_number_array[3];
  uint8_t tempValue;

  int8_t temp_bit = 23; //set start bit
  //decode pin
  //Serial.println("decoding");
  for (uint8_t p = 0; p < 3; p++) {
    temp_pin[p] = 0; //reset pin
    temp_pin_number_array[p] = 0;
    for (int8_t b = 5; b >= 0; b--) {
      bitWrite(temp_pin_number_array[p], b, bitRead(temp_input, temp_bit)); //write bit to char array
      //Serial.print(bitRead(temp_input, temp_bit));
      temp_bit --; //reduce bit value
    }
    //uint8_t temp_response = temp_pin_number_array[p];
    //Serial.print("pin became: "); Serial.println(temp_response);
    if (temp_pin_number_array[p] >= 0 && temp_pin_number_array[p] <= 25) {
      temp_pin[p] = temp_pin_number_array[p] + 'A'; //0-25
    }
    if (temp_pin_number_array[p] >= 26 && temp_pin_number_array[p] <= 51) {
      temp_pin[p] = temp_pin_number_array[p] + 71; //26-51
    }
    if (temp_pin_number_array[p] >= 52 && temp_pin_number_array[p] <= 61) {
      temp_pin[p] = temp_pin_number_array[p] - 4; //52-61
    }
    if (temp_pin_number_array[p] == 62) temp_pin[p] = '+';
    if (temp_pin_number_array[p] == 63) temp_pin[p] = '/';
  }
  //write to single number
  temp_pin_number = temp_pin_number_array[2];
  temp_pin_number |= temp_pin_number_array[1] << 6;
  temp_pin_number |= temp_pin_number_array[0] << 12;
  //Serial.print("Command decoded to: "); Serial.println(temp_pin_number); //uncomment this to get new values

  //decode value
  tempValue = 0;
  for (int8_t b = 5; b >= 0; b--) {
    bitWrite(tempValue, b, bitRead(temp_input, temp_bit)); //write bit to int array
    //Serial.print(bitRead(temp_input, temp_bit));
    temp_bit --; //reduce bit value
  }
  //Serial.println("");

  Serial.print("!WPR Command: "); Serial.print(temp_pin); Serial.print(", Value: "); Serial.println(tempValue);

  //execute command
  uint8_t temp_bool = constrain(tempValue, 0, 1);
  switch (temp_pin_number) {
    case 1106: { //ARS, address reset
        digitalWrite(addressReset, temp_bool);
      } break;
    case 139: { //ACL, address clock
        digitalWrite(addressClock, temp_bool);
      } break;
    case 64820: { //P00
        digitalWrite(primtivePins[0], temp_bool);
      } break;
    case 64821: { //P01
        digitalWrite(primtivePins[1], temp_bool);
      } break;
    case 64822: { //P02
        digitalWrite(primtivePins[2], temp_bool);
      } break;
    case 64823: { //P03
        digitalWrite(primtivePins[3], temp_bool);
      } break;
    case 64824: { //P04
        digitalWrite(primtivePins[4], temp_bool);
      } break;
    case 64825: { //P05
        digitalWrite(primtivePins[5], temp_bool);
      } break;
    case 64826: { //P06
        digitalWrite(primtivePins[6], temp_bool);
      } break;
    case 64827: { //P07
        digitalWrite(primtivePins[7], temp_bool);
      } break;
    case 64828: { //P08
        digitalWrite(primtivePins[8], temp_bool);
      } break;
    case 64829: { //P09
        digitalWrite(primtivePins[9], temp_bool);
      } break;
    case 64884: { //P10
        digitalWrite(primtivePins[10], temp_bool);
      } break;
    case 64885: { //P11
        digitalWrite(primtivePins[11], temp_bool);
      } break;
    case 64886: { //P12
        digitalWrite(primtivePins[12], temp_bool);
      } break;
    case 64887: { //P13
        digitalWrite(primtivePins[13], temp_bool);
      } break;
    case 61579: { //PCL, primitive clock
        digitalWrite(primitiveClock, temp_bool);
      } break;
    case 17216: { //ENA, head enable
        digitalWrite(headEnable, temp_bool);
        //Serial.print("Set");
      } break;
    default: {
        return -1;
      } break;
  }
  return 1; //if you got here, function found a solution
}
