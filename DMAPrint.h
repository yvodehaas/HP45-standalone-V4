/*  OctoWS2811 - High Performance WS2811 LED Display Library
    http://www.pjrc.com/teensy/td_libs_OctoWS2811.html
    Copyright (c) 2013 Paul Stoffregen, PJRC.COM, LLC

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

#ifndef DMAPrint_h
#define DMAPrint_h

#ifdef __AVR__
#error "Sorry, DMAPrint only works on 32 bit Teensy boards.  AVR isn't supported."
#endif

#include <Arduino.h>
#include "DMAChannel.h"

#if TEENSYDUINO < 121
#error "Teensyduino version 1.21 or later is required to compile this library."
#endif
#ifdef __AVR__
#error "DMAPrint does not work with Teensy 2.0 or Teensy++ 2.0."
#endif

class DMAPrint {
  public:
    DMAPrint(uint32_t tempBufferSize, void *portCMem , void *portDMem, void *portCWri, void *portDWri, uint32_t tempFrequency);
    void begin(void);
    void begin(uint32_t tempBufferSize, void *portCMem , void *portDMem, void *portCWri, void *portDWri, uint32_t tempFrequency);

    void set(uint32_t tempPosition, uint8_t tempDataC, uint8_t tempDataD);
    void SetBurst(uint16_t temp_input[22], uint8_t temp_mode);
    void TestHead(uint8_t* temp_nozzle_state, uint8_t* tempAddressState, uint8_t* tempPrimitiveState);
    uint8_t TestDummy(uint8_t temp_dummy);
    void SingleNozzle(uint16_t temp_nozzle);
    int8_t Preheat(uint16_t temp_pulses);
    int8_t Prime(uint16_t temp_pulses);
    uint8_t TestAddress(void); 

    void Burst(void);
    int busy(void);

    void PrimitivePulse(uint16_t tempState);
    void PrimitiveShortPulse(uint16_t tempState);
    void PrimitiveDummyPulse(uint8_t tempDummy);
    void AddressNext(void);
    void AddressReset(void);

    uint32_t GetTSRRaw(uint8_t tempResolution);
    uint32_t Get10XRaw(uint8_t tempResolution);
    uint32_t GetVoltageLogicRaw(uint8_t tempResolution);
    uint32_t GetVoltageHeadRaw(uint8_t tempResolution);
    uint32_t GetVoltageAddressRaw(uint8_t tempResolution);
    uint8_t GetNozzleCheck(void);
    int32_t GetTemperature(void);
    uint32_t GetVoltageLogic(void);
    uint32_t GetVoltageHead(void);
    uint32_t GetVoltageAddress(void);
    
    void SetPrimitiveClock(uint8_t tempState);
    void SetPrimitivePins(uint16_t tempState);
    void SetAddressClock(uint8_t tempState);
    void SetAddressReset(uint8_t tempState);
    void SetEnable(uint8_t tempState);
    void SetEnableTemp(uint8_t tempState);
    void EnableReset();

    void ResetBurst(void);
    uint8_t GetPrimitive(uint16_t temp_nozzle);
    uint8_t GetAddress(uint16_t temp_nozzle);
    int16_t GetNozzle(uint8_t temp_primitive, uint8_t temp_address);
    uint16_t *ConvertB6RawToBurst(uint8_t temp_input[50], uint16_t temp_burst[22]);
    uint16_t *ConvertB6ToggleToBurst(uint8_t temp_input[50], uint16_t temp_burst[22]);
    uint16_t *ConvertB8ToBurst(uint8_t temp_input[38], uint16_t temp_burst[22]);
    void SetDPI(uint16_t temp_dpi);
    void DMASetPulseSplit(uint8_t tempSplit);
    uint8_t DMAGetPulseSplit(void);
    uint8_t GetEnabledState();
    uint8_t WritePinRaw(uint32_t temp_input);


  private:
    static uint16_t dmaBufferSize;
    static void *portCMemory;
    static void *portDMemory;
    static void *portCWrite;
    static void *portDWrite;
    static uint32_t dmaFrequency;
    static DMAChannel dma1, dma2, dma3;
    static void isr(void);
};

#endif
