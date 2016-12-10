/*   TETRIX PRIZM robotics controller cpp File for PRIZM Arduino Library
   	Written by: Paul W. Uttley
	08/22/2016
	Version 1.0

*/

#include <Arduino.h>
#include <avr/wdt.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <Wire.h>
#include "PRIZM.h"

//#include "utility/WSWire.h" // Not the standard Arduino library, but a variant that is supposed to perform better and not lock up


void PRIZM::setMotorSpeedPID(int P, int I, int D){	//=== Change the PID parameters for the constant speed algorithm

  int lobyteP;
  int hibyteP;
  int lobyteI;
  int hibyteI;
  int lobyteD;
  int hibyteD;

  lobyteP  = lowByte(P);
  hibyteP  = highByte(P);
  lobyteI  = lowByte(I);
  hibyteI  = highByte(I);
  lobyteD  = lowByte(D);
  hibyteD  = highByte(D);
   
  Wire.beginTransmission(5);    	    //transmit to I2C address 
  Wire.write(0X56);               	    //send command byte
  Wire.write(hibyteP);                    
  Wire.write(lobyteP);  
  Wire.write(hibyteI);                    
  Wire.write(lobyteI);  
  Wire.write(hibyteD);                    
  Wire.write(lobyteD);  
                  
  Wire.endTransmission();                //end transmission

  delay(10);

}

void PRIZM::setMotorTargetPID(int P, int I, int D){	//=== Change the PID parameters for the targetting (hold position) algorithm

  int lobyteP;
  int hibyteP;
  int lobyteI;
  int hibyteI;
  int lobyteD;
  int hibyteD;

  lobyteP  = lowByte(P);
  hibyteP  = highByte(P);
  lobyteI  = lowByte(I);
  hibyteI  = highByte(I);
  lobyteD  = lowByte(D);
  hibyteD  = highByte(D);
   
  Wire.beginTransmission(5);    	    //transmit to I2C address 
  Wire.write(0X57);               	    //send command byte
  Wire.write(hibyteP);                    
  Wire.write(lobyteP);  
  Wire.write(hibyteI);                    
  Wire.write(lobyteI);  
  Wire.write(hibyteD);                    
  Wire.write(lobyteD);  
                  
  Wire.endTransmission();                //end transmission

  delay(10);

}

int PRIZM::readDCFirmware(){		//==== Request firmware version of DC motor chip

  int byte1;
  int DCversion;
      
  Wire.beginTransmission(5);      //transmit to I2C address 
  Wire.write(0x26);               //send command byte
  Wire.endTransmission();         //end transmission  
  delay(10);
  Wire.requestFrom(5, 1);         //request 1 bytes from I2Caddress
  byte1 = Wire.read();		  //DC chip firmware
  DCversion=byte1;               
  delay(10);
  return DCversion;
}

int PRIZM::readSVOFirmware(){		//==== Request firmware version of Servo motor chip

  int byte1;
  int SVOversion;
      
  Wire.beginTransmission(6);      //transmit to I2C address 
  Wire.write(0x26);               //send command byte
  Wire.endTransmission();         //end transmission  
  delay(10);
  Wire.requestFrom(6, 1);         //request 1 bytes from I2Caddress
  byte1 = Wire.read();		  //SERVO chip firmware
  SVOversion=byte1;               
  delay(10);
  return SVOversion;
}



void PRIZM::PrizmBegin(){  //======= Send a SW reset to all four expansion port I2C addresses (resets everything), and 
  Wire.begin();
  delay(500);               		// last delay is longer to give expansion controllers time to reset
  setGreenLED(HIGH);  			// Turn on when we're reset and ready to go
  while(readStartButton()==0){};   	// wait here for the program start (green) button to be pressed
  setGreenLED(LOW);   			// turn green off

  Wire.beginTransmission(1);    	// DO A sw RESET ON ALL OF THE MOTOR CHIPS AFTER START IS PRESSED TO ZERO ENCODERS 
  Wire.write(0x27);                        
  Wire.endTransmission();                 
  delay(10);
  Wire.beginTransmission(2);     
  Wire.write(0x27);                        
  Wire.endTransmission();                 
  delay(10);	
  Wire.beginTransmission(3);     
  Wire.write(0x27);                        
  Wire.endTransmission();                 
  delay(10);
  Wire.beginTransmission(4);     
  Wire.write(0x27);                        
  Wire.endTransmission(); 
  delay(10);
  Wire.beginTransmission(5);     
  Wire.write(0x27);                        
  Wire.endTransmission();                 
  delay(10);
  Wire.beginTransmission(6);     
  Wire.write(0x27);                        
  Wire.endTransmission();  
  
  delay(1000);     // 1 second delay between time GO button is pushed and program actually starts executing

  Wire.beginTransmission(1);    	// Send an "Enable" Byte to the DC and Servo controller chips of all controllers
  Wire.write(0x25);                  // I added an enable command so that the robot won't immediately start to move if the PrizmBegin statment is accidentally left out of code      
  Wire.endTransmission();                 
  delay(10);
  Wire.beginTransmission(2);     
  Wire.write(0x25);                        
  Wire.endTransmission();                 
  delay(10);	
  Wire.beginTransmission(3);     
  Wire.write(0x25);                        
  Wire.endTransmission();                 
  delay(10);
  Wire.beginTransmission(4);     
  Wire.write(0x25);                        
  Wire.endTransmission(); 
  delay(10);
  Wire.beginTransmission(5);     
  Wire.write(0x25);                        
  Wire.endTransmission();                 
  delay(10);
  Wire.beginTransmission(6);     
  Wire.write(0x25);                        
  Wire.endTransmission();  
	
  delay(100);
  
}

void PRIZM::PrizmEnd(){  //======= Send a SW reset to all four expansion port I2C addresses (resets everything)
  Wire.beginTransmission(1);     
  Wire.write(0x27);                        
  Wire.endTransmission();                 
  delay(10);
  Wire.beginTransmission(2);     
  Wire.write(0x27);                        
  Wire.endTransmission();                 
  delay(10);
  Wire.beginTransmission(3);     
  Wire.write(0x27);                        
  Wire.endTransmission();                 
  delay(10);
  Wire.beginTransmission(4);     
  Wire.write(0x27);                        
  Wire.endTransmission(); 
  delay(10);
  Wire.beginTransmission(5);     
  Wire.write(0x27);                        
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(6);     
  Wire.write(0x27);                        
  Wire.endTransmission(); 
  delay(10); 

  wdt_enable(WDTO_15MS);  // set the wdt to timeout after 15ms after which automatically resets processor
  for(;;)                 
    {                       
    }                   
    
}

int PRIZM::readBatteryVoltage(){

  int Bvoltage = analogRead(0)*2;
  return Bvoltage;
}

int PRIZM::readLineSensor(int pin){     // Can sense black or white (follow the edge of a black line on a white surface)
  int BWstate; // black or white 
  pinMode(pin, INPUT);
  if(HIGH == digitalRead(pin)){BWstate = 1;} else {BWstate = 0;}

  return BWstate;
}

int PRIZM::readSonicSensorCM(int pin){   // Returns distance of object from sensor in Centimeters

  int duration;
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delayMicroseconds(2);
  digitalWrite(pin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pin,LOW);
  pinMode(pin,INPUT);
  duration = pulseIn(pin,HIGH);

  return duration/29/2; // convert time of echo to centimeters distance
  
}

int PRIZM::readSonicSensorIN(int pin){   // Returns distance of object from sensor in Inches

  int duration;
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delayMicroseconds(2);
  digitalWrite(pin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pin,LOW);
  pinMode(pin,INPUT);
  duration = pulseIn(pin,HIGH);

  return duration/74/2; // convert time of echo to centimeters distance
  
}


void PRIZM::setGreenLED (int state){
  pinMode(7, OUTPUT); //===== GREEN LED is on pin 7
  if (state==1){digitalWrite(7, HIGH);}
  if (state==0){digitalWrite(7, LOW);}
}

void PRIZM::setRedLED (int state){
  pinMode(6, OUTPUT); //===== RED LED is on pin 6
  if (state==1){digitalWrite(6, HIGH);}
  if (state==0){digitalWrite(6, LOW);}
}

int PRIZM::readStartButton(){       //============== function returns; unpressed button == 0; pressed button == 1
  pinMode(8, INPUT); // Button is on pin 8; unpressed = high, pressed = low
  int StartBtn = digitalRead(8);
  StartBtn = !StartBtn; // toggle variable to make sense;
  return StartBtn;
}

void PRIZM::setServoSpeed (int channel, int servospeed){   //=========== function for setting servo speeds individually

  if(channel==1){channel= 0x28;}
  if(channel==2){channel= 0x29;}
  if(channel==3){channel= 0x2A;}
  if(channel==4){channel= 0x2B;}
  if(channel==5){channel= 0x2C;}
  if(channel==6){channel= 0x2D;}

  Wire.beginTransmission(6);     		 //transmit to I2C address 
  Wire.write(channel);                	 //send command byte
  Wire.write(servospeed);                  //set servo speed
  Wire.endTransmission();                  //end transmission

   delay(10);

   
}

void PRIZM::setServoSpeeds (int servospeed1, int servospeed2, int servospeed3, int servospeed4, int servospeed5, int servospeed6){   // function to set all servo speeds at once
   
  Wire.beginTransmission(6);     //transmit to I2C address 
  Wire.write(0x2E);                       //send command byte 
  Wire.write(servospeed1);                //set each servo speed all at once
  Wire.write(servospeed2);
  Wire.write(servospeed3);
  Wire.write(servospeed4);
  Wire.write(servospeed5);
  Wire.write(servospeed6);
  Wire.endTransmission();                 //end transmission

   delay(10);
  
}

void PRIZM::setServoPosition (int channel, int servoposition){   //function to set servo positions individually
    
  if(channel==1){channel= 0x2F;}
  if(channel==2){channel= 0x30;}
  if(channel==3){channel= 0x31;}
  if(channel==4){channel= 0x32;}
  if(channel==5){channel= 0x33;}
  if(channel==6){channel= 0x34;}
  
  Wire.beginTransmission(6);     	  //transmit to I2C address 
  Wire.write(channel);                 	  //send command byte 
  Wire.write(servoposition);                //set servo position
  Wire.endTransmission();                   //end transmission  

  delay(10);
  
}


void PRIZM::setServoPositions (int servoposition1,int servoposition2,int servoposition3,int servoposition4,int servoposition5,int servoposition6){
   
  Wire.beginTransmission(6);         		   //transmit to I2C address 
  Wire.write(0x35);                           //send command byte 
  Wire.write(servoposition1);                 //set each servo position all at once
  Wire.write(servoposition2);
  Wire.write(servoposition3);
  Wire.write(servoposition4);
  Wire.write(servoposition5);
  Wire.write(servoposition6);
  Wire.endTransmission();                   //end transmission  

   delay(10);
  
}

void PRIZM::setCRServoState (int channel, int servospeed){   // function to set CR servos speed and direction -100|0|100

  if(channel==1){channel= 0x36;}   // CRservo 1
  if(channel==2){channel= 0x37;}   // CRservo 2
  
  Wire.beginTransmission(6);      //transmit to I2C address 
  Wire.write(channel);                 //send command byte
  Wire.write(servospeed);                  //set servo position - CR servo -100 (CCW) | 0 (stop) | 100 (CW)
  Wire.endTransmission();                  //end transmission 

  delay(10);
  
}

int PRIZM::readServoPosition (int channel){

  int readServoPosition;    // return value variable
   
  if(channel==1){channel= 0x38;}
  if(channel==2){channel= 0x39;}
  if(channel==3){channel= 0x3A;}
  if(channel==4){channel= 0x3B;}
  if(channel==5){channel= 0x3C;}
  if(channel==6){channel= 0x3D;}
     
  Wire.beginTransmission(6);      //transmit to I2C address 
  Wire.write(channel);                 	//send command byte 
  Wire.endTransmission();                  //end transmission  
  delay(10);
  Wire.requestFrom(6, 1);        //request 1 byte from I2Caddress
  readServoPosition = Wire.read();

  delay(10);
  
  return readServoPosition;
 
}



void PRIZM::setMotorPower(int channel, int power)	// transmit power to select DC motor channel on # controller
{

	if(channel==1){channel = 0x40;}   // DC channel 1
  	if(channel==2){channel = 0x41;}   // DC channel 2
 
  	Wire.beginTransmission(5);     		//transmit to I2C address 
  	Wire.write(channel);                 //send command byte 
  	Wire.write(power);                   //transmit power level 
  	Wire.endTransmission();              //end transmission

  
}

void PRIZM::setMotorPowers (int power1, int power2){     //power only Block Command for Motor 1 and 2 (both in one transmission)
  
  Wire.beginTransmission(5);    	    //transmit to I2C address 
  Wire.write(0x42);                      //send command byte
  Wire.write(power1);                    //transmit power1 level -100 | 100  0 = stop coast; -128 = stop brake
  Wire.write(power2);                    //transmit power2 level -100 | 100  0 = stop coast; -128 = stop brake
  Wire.endTransmission();                //end transmission

  delay(10);
  
}

void PRIZM::setMotorSpeed (int channel, long Mspeed){      // === set speed of each DC motor == requires an installed encoder to do the PID

  int lobyte;
  int hibyte;

  lobyte  = lowByte(Mspeed);
  hibyte  = highByte(Mspeed);
    
  if(channel==1){channel= 0x43;}   // DC channel 1
  if(channel==2){channel= 0x44;}   // DC channel 2

  Wire.beginTransmission(5);    //transmit to I2C address 
  Wire.write(channel);               	    //send command byte
  Wire.write(hibyte);                    //MAX DPS should be +/- 720
  Wire.write(lobyte);                    
  Wire.endTransmission();                //end transmission

  delay(10);

  
}

void PRIZM::setMotorSpeeds (long Mspeed1, long Mspeed2){      // === BLOCK write to set speeds of both motors at once == encoder must be installed to do PID

  int lobyte1;
  int hibyte1;

  int lobyte2;
  int hibyte2;

  lobyte1  = lowByte(Mspeed1);
  hibyte1  = highByte(Mspeed1);

  lobyte2  = lowByte(Mspeed2);
  hibyte2  = highByte(Mspeed2);
 
  Wire.beginTransmission(5);    //transmit to I2C address 
  Wire.write(0x45);                      //send command byte
  Wire.write(hibyte1);                   //MAX DPS is +/- 720 motor channel 1
  Wire.write(lobyte1);     
  Wire.write(hibyte2);                   //MAX DPS is +/- 720 motor channel 2
  Wire.write(lobyte2);                  
  Wire.endTransmission();                //end transmission

  delay(10);
}

void PRIZM::setMotorTarget (int channel, long Mspeed, long Mtarget){      // === set speed and encoder target of each DC motor == requires an installed encoder to do the PID

  int lobyte;
  int hibyte;

  lobyte  = lowByte(Mspeed);
  hibyte  = highByte(Mspeed);

  byte four  = (Mtarget);
  byte three = (Mtarget>>8);
  byte two   = (Mtarget>>16);
  byte one   = (Mtarget>>24);
  
  if(channel==1){channel= 0x46;}   // DC channel 1
  if(channel==2){channel= 0x47;}   // DC channel 2

  Wire.beginTransmission(5);       //transmit to I2C address 
  Wire.write(channel);                //send command byte
  Wire.write(hibyte);                    // MAX DPS is +/- 720
  Wire.write(lobyte);      
  Wire.write(one);
  Wire.write(two);
  Wire.write(three);
  Wire.write(four);
  Wire.endTransmission();                //end transmission

  delay(10);
  
}

void PRIZM::setMotorTargets (long Mspeed1, long Mtarget1, long Mspeed2, long Mtarget2){      // === BLOCK WRITE set speed and encoder target of both DC motors == requires an installed encoder to do the PID

  int lobyte1;
  int hibyte1;

  int lobyte2;
  int hibyte2;

  lobyte1  = lowByte(Mspeed1);
  hibyte1  = highByte(Mspeed1);

  lobyte2  = lowByte(Mspeed2);
  hibyte2  = highByte(Mspeed2);

  byte four1  = (Mtarget1);
  byte three1 = (Mtarget1>>8);
  byte two1   = (Mtarget1>>16);
  byte one1   = (Mtarget1>>24);

  byte four2  = (Mtarget2);
  byte three2 = (Mtarget2>>8);
  byte two2   = (Mtarget2>>16);
  byte one2   = (Mtarget2>>24);
  
  Wire.beginTransmission(5);    //transmit to I2C address 
  Wire.write(0x48);                      //send command byte
  Wire.write(hibyte1);                   //MAX DPS is +/- 720
  Wire.write(lobyte1);      
  Wire.write(one1);
  Wire.write(two1);
  Wire.write(three1);
  Wire.write(four1);
  Wire.write(hibyte2);                    // MAX DPS is +/- 720
  Wire.write(lobyte2);      
  Wire.write(one2);
  Wire.write(two2);
  Wire.write(three2);
  Wire.write(four2);
  Wire.endTransmission();                //end transmission

  delay(10);
  
}

void PRIZM::setMotorDegree (int channel, long Mspeed, long Mdegrees){      // === set speed and encoder target of each DC motor in DEGREES  == requires an installed encoder to do the PID

  int lobyte;
  int hibyte;

  lobyte  = lowByte(Mspeed);
  hibyte  = highByte(Mspeed);

  byte four  = (Mdegrees);
  byte three = (Mdegrees>>8);
  byte two   = (Mdegrees>>16);
  byte one   = (Mdegrees>>24);
  
  if(channel==1){channel= 0x58;}   // DC channel 1 degrees register
  if(channel==2){channel= 0x59;}   // DC channel 2 degrees register

  Wire.beginTransmission(5);      //transmit to I2C address 
  Wire.write(channel);                	//send command byte
  Wire.write(hibyte);                     // MAX DPS is +/- 720
  Wire.write(lobyte);      
  Wire.write(one);
  Wire.write(two);
  Wire.write(three);
  Wire.write(four);
  Wire.endTransmission();                //end transmission

  delay(10);
  
}


void PRIZM::setMotorDegrees (long Mspeed1, long Mdegrees1, long Mspeed2, long Mdegrees2){      // === BLOCK WRITE set speed and encoder target in DEGREES of both DC motors == requires an installed encoder to do the PID

  int lobyte1;
  int hibyte1;

  int lobyte2;
  int hibyte2;

  lobyte1  = lowByte(Mspeed1);
  hibyte1  = highByte(Mspeed1);

  lobyte2  = lowByte(Mspeed2);
  hibyte2  = highByte(Mspeed2);

  byte four1  = (Mdegrees1);
  byte three1 = (Mdegrees1>>8);
  byte two1   = (Mdegrees1>>16);
  byte one1   = (Mdegrees1>>24);

  byte four2  = (Mdegrees2);
  byte three2 = (Mdegrees2>>8);
  byte two2   = (Mdegrees2>>16);
  byte one2   = (Mdegrees2>>24);
  
  Wire.beginTransmission(5);   		    //transmit to I2C address 
  Wire.write(0x5A);                      //send command byte for block write DEGREES register
  Wire.write(hibyte1);                   //MAX DPS is +/- 720
  Wire.write(lobyte1);      
  Wire.write(one1);
  Wire.write(two1);
  Wire.write(three1);
  Wire.write(four1);
  Wire.write(hibyte2);                    // MAX DPS is +/- 720
  Wire.write(lobyte2);      
  Wire.write(one2);
  Wire.write(two2);
  Wire.write(three2);
  Wire.write(four2);
  Wire.endTransmission();                //end transmission

  delay(10);
  
}

long PRIZM::readEncoderCount (int channel){   // ============================= READ ENCODER DATA COUNTS ====================================

  unsigned long eCount;    // return value variable. We have to pass this an unsigned into Arduino.

  byte byte1;
  byte byte2;
  byte byte3;
  byte byte4;
  
  
  if(channel==1){channel= 0x49;}       // channel 1 encoder FOR count value
  if(channel==2){channel= 0x4A;}       // channel 2 encoder FOR count value
         
  Wire.beginTransmission(5);      //transmit to I2C address 
  Wire.write(channel);                  //send command byte
  Wire.endTransmission();                  //end transmission  
  delay(10);
  Wire.requestFrom(5, 4);         //request 4 bytes from I2Caddress
  byte1 = Wire.read();
  byte2 = Wire.read();
  byte3 = Wire.read();
  byte4 = Wire.read();  


  eCount = byte1;
  eCount = (eCount*256)+byte2;
  eCount = (eCount*256)+byte3;
  eCount = (eCount*256)+byte4;

  delay(10);
 
  return eCount;

}

long PRIZM::readEncoderDegrees (int channel){   // ============================= READ ENCODER DATA DEGREES ====================================

  unsigned long eCount;    // return value variable. We have to pass this an unsigned into Arduino.


  byte byte1;
  byte byte2;
  byte byte3;
  byte byte4;
  
  if(channel==1){channel= 0x5B;}       // channel 1 encoder FOR degrees
  if(channel==2){channel= 0x5C;}       // channel 2 encoder FOR degrees
  
  Wire.beginTransmission(5);      //transmit to I2C address 
  Wire.write(channel);                  	//send command byte
  Wire.endTransmission();                  //end transmission  
  delay(10);
  Wire.requestFrom(5, 4);         //request 4 bytes from I2Caddress
  byte1 = Wire.read();
  byte2 = Wire.read();
  byte3 = Wire.read();
  byte4 = Wire.read();  

  eCount = byte1;
  eCount = (eCount*256)+byte2;
  eCount = (eCount*256)+byte3;
  eCount = (eCount*256)+byte4;

  delay(10);
 
  return eCount;

}

void PRIZM::resetEncoder (int channel){    // =================== RESET ENCODERS 1 or 2 =============================

  
  if(channel==1){channel= 0x4C;}       // channel 1 encoder reset command
  if(channel==2){channel= 0x4D;}       // channel 2 encoder reset command
  
  Wire.beginTransmission(5);      //transmit to I2C address 
  Wire.write(channel);                  //send command byte
  Wire.endTransmission();                  //end transmission  

  delay(10);
  
}

void PRIZM::resetEncoders(){			// ================== Reset BOTH Encoders at once =========================

  Wire.beginTransmission(5);      //transmit to I2C address 
  Wire.write(0x4E);               //send command byte
  Wire.endTransmission();         //end transmission  

  delay(10);

}   

int PRIZM::readMotorBusy (int channel){

  int byte1;
  int MotorStatus;
  
  if(channel==1){channel= 0x4F;}       // channel 1 busy flag
  if(channel==2){channel= 0x50;}       // channel 2 busy flag
  
  Wire.beginTransmission(5);      //transmit to I2C address 
  Wire.write(channel);                  //send command byte
  Wire.endTransmission();                  //end transmission  
  delay(10);
  Wire.requestFrom(5, 1);         //request 1 bytes from I2Caddress
  byte1 = Wire.read();

  MotorStatus=byte1;                      // = 1 Motor is busy | = 0 Motor has reached target and not busy

  delay(10);
  
  return MotorStatus;
  
}

int PRIZM::readMotorCurrent (int channel){

  int Mcurrent;

  byte byte1;
  byte byte2;
  
  if(channel==1){channel= 0x54;}       // read DC motor 1 current
  if(channel==2){channel= 0x55;}       // read DC motor 2 current
  
  Wire.beginTransmission(5);      //transmit to I2C address 
  Wire.write(channel);                  //send command byte
  Wire.endTransmission();                  //end transmission  

  delay(10);

  Wire.requestFrom(5, 2);         //request 2 bytes from I2Caddress
  byte1 = Wire.read();
  byte2 = Wire.read();
  
  Mcurrent = byte1;
  Mcurrent = (Mcurrent*256)+byte2;
  
  delay(10);

  return (Mcurrent);     // return Mcurrent in milliamps
   
}

void PRIZM::setMotorInvert (int channel, int invert){

  if(channel==1){channel= 0x51;}       // channel 1 
  if(channel==2){channel= 0x52;}       // channel 2 
  
  Wire.beginTransmission(5);      //transmit to I2C address 
  Wire.write(channel);                  //send command byte, which motor to invert/no invert
  Wire.write(invert);                      //0 = no invert; 1 = invert;
  Wire.endTransmission();                  //end transmission 

  delay(10);
  
}



//=========THE END - FINALLY!===========================================================================================================================================================================================


