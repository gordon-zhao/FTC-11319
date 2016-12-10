/* 	Header file for TETRIX PRIZM robotics controller Arduino Library
   	Written by: Paul W. Uttley
	08/22/2016
	Version 1.0

*/

#include <Arduino.h>


#ifndef PRIZM_H
#define PRIZM_H

//#include <avr/wdt.h>
//#include <inttypes.h>
//#include <stdio.h>
//#include <string.h>
//#include "PRIZM.h"

//#include "utility/WSWire.h" // Not the standard Arduino library, but a variant that is supposed to perform better and not lock up



class PRIZM
{
	public:
		
		void setMotorPower(int channel, int power);
		void setMotorPowers(int power1,int power2);

		void setMotorSpeed (int channel, long Mspeed);
		void setMotorSpeeds (long Mspeed1, long Mspeed2);
		void setMotorTarget (int channel, long Mspeed, long Mtarget);
		void setMotorTargets (long Mspeed1, long Mtarget1, long Mspeed2, long Mtarget2);
		void setMotorDegree (int channel, long Mspeed, long Mdegrees);
		void setMotorDegrees (long Mspeed1, long Mdegrees1, long Mspeed2, long Mdegrees2);		
		void setMotorInvert (int channel, int invert);
		int readMotorBusy (int channel);
		int readMotorCurrent (int channel);


		long readEncoderCount (int channel);
		long readEncoderDegrees (int channel);
		void resetEncoder (int channel);
		void resetEncoders (void);

		void setMotorSpeedPID  (int P, int I, int D);
		void setMotorTargetPID (int P, int I, int D);

		int readLineSensor(int pin);
		int readSonicSensorCM(int pin);
		int readSonicSensorIN(int pin);

		int readBatteryVoltage(void);
		
		void PrizmBegin(void);
		void PrizmEnd(void);
		int readDCFirmware(void);
		int readSVOFirmware(void);

		void setGreenLED(int state);
		void setRedLED(int state);

		int  readStartButton(void);

		void setServoSpeed(int channel, int servospeed);
		void setServoSpeeds(int servospeed1, int servospeed2, int servospeed3, int servospeed4, int servospeed5, int servospeed6);
		void setServoPosition (int channel, int servoposition);
		void setServoPositions (int servoposition1,int servoposition2,int servoposition3,int servoposition4,int servoposition5,int servoposition6);
		void setCRServoState (int channel, int servospeed);
		int readServoPosition (int channel);
		
		
	private:


		
		
};

#endif







