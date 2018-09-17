/*  Project: Farmbot CANbus master controller
 *  Author: Phobos Industries - Andrew Filmer
 *  Date: 30/08/18
 *  Version: 0.01
 *  Dependencies:
 *  // CAN library:
 *  // Copyright (c) Sandeep Mistry. All rights reserved.
 *  // Licensed under the MIT license. See LICENSE file in the project root for full license information.
 *  // https://github.com/sandeepmistry/arduino-CAN
 */

#ifndef CANBUSFUNCTIONS_H_
#define CANBUSFUNCTIONS_H_

#ifndef BUFFER_SIZE
#define BUFFER_SIZE 8
#endif

#include "Arduino.h"
#include "pins.h"
#include "Config.h"
#include "StepperControl.h"
//#include "StepperControlAxis.h"
//#include "StepperControlEncoder.h"
#include <stdio.h>
#include <stdlib.h>
#include <SPI.h>
#include <CAN.h>

class CANbusFunctions
{

public:

	// Variables
	uint8_t CANincomingAddr[BUFFER_SIZE] = {0};
	uint8_t CANvalue[BUFFER_SIZE][4] = {{0}};
	long CANencoderVal[BUFFER_SIZE] = {0};
	uint8_t CANinstruction[BUFFER_SIZE] = {0};
	bool CANmessageReceived = false;
	uint8_t CANbufferIndex = 0;
	uint8_t CANworkingIndex = 0;
	bool CANbusWarning[4] = {false};
	bool CANbusError[4] = {false};
	uint8_t CANpacket[4] = {0};

	// Buffer variables
	uint8_t incomingAddressBuffer[BUFFER_SIZE] = {0};
	uint8_t incomingCANinstruction[BUFFER_SIZE] = {0};
	uint8_t incomingValue[BUFFER_SIZE][4] = {{0}};
  
	// Functions
	void CANbusInit();
	void decodeCAN();
	void encodeCAN(long value);
	void CANinterrupt(int packetSize);
	void errorChecking();
	void requestEncoderUpdate(uint8_t addr);

private:

	bool x1ModuleOK = false;
	bool x2ModuleOK = false;
	bool yModuleOK = false;
	bool zModuleOK = false;

};

#endif /* CANBUSFUNCTIONS_H_ */
