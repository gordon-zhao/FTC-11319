/*  PRIZM Controller example program
 *  This program will move the PRIZM TaskBot forward for 3 seconds, stop and end program.
 *  author PWU 08/05/2016
 */

  #include <PRIZM.h>     // include PRIZM library
  PRIZM prizm;           // instantiate a PRIZM object “prizm” so we can use its functions

void setup() {

  prizm.PrizmBegin();    // initialize PRIZM

  prizm.setMotorInvert(1,1);     // invert the direction of DC Motor 1
                                 // to harmonize the direction of opposite facing drive motors
  pinMode(15, INPUT);
  Serial.begin(9600);
  Serial.println("Initialized!");
  
}

void loop() {
  /*
  prizm.setMotorPowers(50,50);   // turn Motors 1 and 2 on at 50% power
  delay(3000);                   // wait here for 3 seconds while motors are spinning
  prizm.PrizmEnd();              // end program and reset PRIZM
  */
  int val = digitalRead(15);
  if (val == HIGH){
    prizm.setMotorPowers(100,0);
    Serial.println("Run!");
  }
  else if (val == LOW){
    prizm.setMotorPowers(0,0);
    Serial.println("STOP!");
  };
}














