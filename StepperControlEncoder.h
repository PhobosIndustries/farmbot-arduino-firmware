/*
 * StepperControlEncoder.h
 *
 *  Created on: 29 okt 2015
 *      Author: Tim Evers
 */

#ifndef STEPPERCONTROLENCODER_H_
#define STEPPERCONTROLENCODER_H_

#include "Arduino.h"
//#include "CurrentState.h"
//#include "ParameterList.h"
#include "pins.h"
#include "Config.h"
#include <stdio.h>
#include <stdlib.h>
#include <SPI.h>
#include <CAN.h>

class StepperControlEncoder
{

public:
  StepperControlEncoder();

  void loadPinNumbers(int channelA, int channelB, int channelAQ, int channelBQ);
  void loadSettings(int encType, int scaling, int invert);

  // Load the id for the motor dynamics lab encoder
  void loadMdlEncoderId(MdlSpiEncoders encoder);

  void setPosition(long newPosition);
  void setPositionByCAN(long newPosition);
  long currentPosition();
  long currentPositionRaw();

  void checkEncoder(bool channelA, bool channelB, bool channelAQ, bool channelBQ);
  void processEncoder();
  void readChannels();
  void setChannels(bool channelA, bool channelB, bool channelAQ, bool channelBQ);
  void shiftChannels();
  void test();
  #if defined(RAMPS_V14_CANBUS)
    // Load the id for CANbus based encoder module
    void loadCANbusEncoderId(int16_t encoder);
    void loadMicrosteppingValue(uint8_t microsteps);
  #endif

private:
  // pin settings
  int pinChannelA;
  int pinChannelAQ;
  int pinChannelB;
  int pinChannelBQ;

  // channels
  bool prvValChannelA;
  bool prvValChannelB;
  bool curValChannelA;
  bool curValChannelB;

  bool readChannelA;
  bool readChannelAQ;
  bool readChannelB;
  bool readChannelBQ;

  // encoder
  long position;
  int scalingFactor;
  int encoderType;
  int encoderInvert;

  MdlSpiEncoders mdlEncoder = _MDL_X1;
  
  #if defined(RAMPS_V14_CANBUS)
    int16_t CANbusEncoder = ENC_X2_CAN_ID;
    uint8_t microStepping = X2_MICROSTEP;
    uint8_t microSteppingCounter = 0;
    uint32_t CANbusLastRequest = 0;
    uint16_t CANbusRequestInterval = CANBUS_REQUEST_INTERVAL;
  #endif

};

#endif /* STEPPERCONTROLENCODER_H_ */