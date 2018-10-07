#include "StepperControlEncoder.h"
#include "CANbusFunctions.h"

StepperControlEncoder::StepperControlEncoder()
{
  //lastCalcLog	= 0;

  pinChannelA = 0;
  pinChannelB = 0;

  position = 0;
  encoderType = 0; // default type
  scalingFactor = 10000;

  curValChannelA = false;
  curValChannelA = false;
  prvValChannelA = false;
  prvValChannelA = false;

  readChannelA = false;
  readChannelAQ = false;
  readChannelB = false;
  readChannelBQ = false;

  mdlEncoder = _MDL_X1;
  
  #ifdef RAMPS_V14_CANBUS
	CANbusEncoder = ENC_X2_CAN_ID;
  microStepping = X2_MICROSTEP;
  microSteppingCounter = 0;
  CANbusLastRequest = 0;
  CANbusRequestInterval = CANBUS_REQUEST_INTERVAL;
  #endif
}

void StepperControlEncoder::test()
{
  /*
                Serial.print("R88 ");
                Serial.print("position ");
                Serial.print(position);
                Serial.print(" channel A ");
                Serial.print(prvValChannelA);
                Serial.print(" -> ");
                Serial.print(curValChannelA);
                Serial.print(" channel B ");
                Serial.print(prvValChannelB);
                Serial.print(" -> ");
                Serial.print(curValChannelB);
                Serial.print("\r\n");
*/
}

void StepperControlEncoder::loadPinNumbers(int channelA, int channelB, int channelAQ, int channelBQ)
{
  pinChannelA = channelA;
  pinChannelB = channelB;
  pinChannelAQ = channelAQ;
  pinChannelBQ = channelBQ;

  readChannels();
  shiftChannels();
}

void StepperControlEncoder::loadSettings(int encType, int scaling, int invert)
{
  encoderType = encType;
  scalingFactor = scaling;
  if (invert == 1)
  {
    encoderInvert = -1;
  }
  else
  {
    encoderInvert = 1;
  }

//  encoderType = 0; // TEVE 2017-04-20 Disabling the differential channels. They take too much time to read.
}

void StepperControlEncoder::loadMdlEncoderId(MdlSpiEncoders encoder)
{
  mdlEncoder = encoder;
}

void StepperControlEncoder::setPosition(long newPosition)
{
  #if defined(RAMPS_V14) || defined(FARMDUINO_V10)
    position = newPosition;
  #endif

  #if defined(FARMDUINO_V14)
    if (newPosition == 0)
    {
      position = newPosition;

      const byte reset_cmd = 0x00;

      digitalWrite(NSS_PIN, LOW);
      SPI.transfer(reset_cmd | (mdlEncoder << mdl_spi_encoder_offset));
      digitalWrite(NSS_PIN, HIGH);
    }
  #endif

  #if defined(RAMPS_V14_CANBUS)

    position = newPosition;
    
    //Code below is only to update remote modules with a value set by Gcode. Current Farmbot Gcode specifies only 0 can be set. The following code can be used to update a non zero value by Gcode in the future
    
    CANbusFunctions::getInstance()->encodeCAN(newPosition);

    // Update remote CANbus module (remoteAddress) -> (instruction) -> (packet[], length) -> (transmit)
    CAN.beginPacket(CANbusEncoder);
    CAN.write('S'); // Set position
    CAN.write(CANbusFunctions::getInstance()->CANpacket,4);
    CAN.endPacket();
    
    /*
    CAN.beginPacket(CANbusEncoder);
    CAN.write('I');
    CAN.endPacket();
    */
    
  #endif
}

#if defined (RAMPS_V14_CANBUS)
  void StepperControlEncoder::setPositionByCAN(long newPosition)
  {
    position = newPosition;
  }
#endif

long StepperControlEncoder::currentPosition()
{


  // Apply scaling to the output of the encoder, except when scaling is zero or lower
  if (scalingFactor == 10000 || scalingFactor <= 0)
  {
    return position * encoderInvert;
  }
  else
  {
    #if defined(FARMDUINO_V14)
      return position * scalingFactor / 40000 * encoderInvert;
    #endif
    #if defined(RAMPS_V14_CANBUS)
      return position * scalingFactor / 10000 * encoderInvert * microStepping;
    #endif
    return position * scalingFactor / 10000 * encoderInvert;
  }
}

long StepperControlEncoder::currentPositionRaw()
{
    return position * encoderInvert;
}

void StepperControlEncoder::checkEncoder(bool channelA, bool channelB, bool channelAQ, bool channelBQ)
{
  #if defined(RAMPS_V14) || defined(FARMDUINO_V10)
    shiftChannels();
    setChannels(channelA, channelB, channelAQ, channelBQ);
    processEncoder();
  #endif

  #if defined(FARMDUINO_V14) || defined(RAMPS_V14_CANBUS)
    processEncoder();
  #endif

}


/* Check the encoder channels for movement according to this specification
                    ________            ________
Channel A          /        \          /        \
             _____/          \________/          \________
                         ________            ________
Channel B               /        \          /        \
             __________/          \________/          \____
                                   __
Channel I                         /  \
             ____________________/    \___________________

rotation ----------------------------------------------------->

*/

void StepperControlEncoder::processEncoder()
{

  #if defined(RAMPS_V14) || defined(FARMDUINO_V10)

    // Detect edges on the A channel when the B channel is high
    if (curValChannelB == true && prvValChannelA == false && curValChannelA == true)
    {
      //delta--;
      position--;
    }
    if (curValChannelB == true && prvValChannelA == true && curValChannelA == false)
    {
      //delta++;
      position++;
    }

  #endif

  // If using farmduino, revision 1.4, use the SPI interface to read from the Motor Dynamics Lab chip
  #if defined(FARMDUINO_V14)
    const byte read_cmd = 0x0F;
    int readSize = 4;
    long encoderVal = 0;

    digitalWrite(NSS_PIN, LOW);
    SPI.transfer(read_cmd | (mdlEncoder << mdl_spi_encoder_offset));
    delayMicroseconds(10);

    for (int i = 0; i < readSize; ++i)
    {
      encoderVal <<= 8;
      encoderVal |= SPI.transfer(0x01);
    }

    digitalWrite(NSS_PIN, HIGH);
    position = encoderVal;
  #endif
  
  // If using RAMPS V1.4 with CANbus communication to remote encoder modules (CAN via SPI)
  #if defined(RAMPS_V14_CANBUS)
    uint8_t readSize = 4;
    uint32_t _timeNow = millis();

    // When an axis is active only check position once every full cycle through the microstepping value (to prevent spamming of the CANbus)    
    if(CANbusFunctions::getInstance()->CANaxisActive == true) // If an axis is active - update encoder position often
    {
      if(microSteppingCounter >= microStepping - 1)
      {
        // Reset counter
        microSteppingCounter = 0;
        
        if((_timeNow - CANbusLastRequest) > CANbusRequestInterval)
        {
          // Update time
          CANbusLastRequest = _timeNow;
    
          // Mark as ready to request update
          CANbusFunctions::getInstance()->CANupdateNow = true;
        }
      }
      else
      {
        // Increment the counter instead
        microSteppingCounter++;
      }
    }
    else  // Axis is idle - update less often?
    {
      if((_timeNow - CANbusLastRequest) > CANbusRequestInterval)
      {
        // Update time
        CANbusLastRequest = _timeNow;
  
        // Mark as ready to request update
        CANbusFunctions::getInstance()->CANupdateNow = true;
      }
    }
    
    // Ask for update
    if(CANbusFunctions::getInstance()->CANupdateNow = true)
    {
      // Send position request code to remote axis module (remoteAddress, DLC[32bit], RTR)
      CAN.beginPacket(CANbusEncoder, readSize, true);
      CAN.endPacket();

      // Reset
      CANbusFunctions::getInstance()->CANupdateNow == false;
    }
    

  #endif

}

void StepperControlEncoder::readChannels()
{

#if defined(RAMPS_V14) || defined(FARMDUINO_V10)
  // read the new values from the coder

  readChannelA = digitalRead(pinChannelA);
  readChannelB = digitalRead(pinChannelB);

  if (encoderType == 1)
  {
    readChannelAQ = digitalRead(pinChannelAQ);
    readChannelBQ = digitalRead(pinChannelBQ);

    // differential encoder
    if ((readChannelA ^ readChannelAQ) && (readChannelB ^ readChannelBQ))
    {
      curValChannelA = readChannelA;
      curValChannelB = readChannelB;
    }
  }
  else
  {

    // encoderType <= 0
    // non-differential incremental encoder
    curValChannelA = readChannelA;
    curValChannelB = readChannelB;
  }
#endif

}

void StepperControlEncoder::setChannels(bool channelA, bool channelB, bool channelAQ, bool channelBQ)
{
  // read the new values from the coder

  if (encoderType == 1)
  {
    // differential encoder
    if ((channelA ^ channelAQ) && (channelB ^ channelBQ))
    {
      curValChannelA = channelA;
      curValChannelB = channelB;
    }
  }
  else
  {
    // encoderType <= 0
    // non-differential incremental encoder
    curValChannelA = channelA;
    curValChannelB = channelB;
  }
}

void StepperControlEncoder::shiftChannels()
{

  // Save the current enoder status to later on compare with new values

  prvValChannelA = curValChannelA;
  prvValChannelB = curValChannelB;
}

#if defined(RAMPS_V14_CANBUS)

  void StepperControlEncoder::loadCANbusEncoderId(int16_t encoder) {
    CANbusEncoder = encoder;
  }

  void StepperControlEncoder::loadMicrosteppingValue(uint8_t microsteps) {
    microStepping = microsteps;
  }

#endif
