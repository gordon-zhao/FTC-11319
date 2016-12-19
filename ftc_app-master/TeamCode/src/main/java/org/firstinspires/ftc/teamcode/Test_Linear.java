/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.LightSensor;

import java.util.concurrent.TimeUnit;

@TeleOp(name="Manual", group="Test")  // @Autonomous(...) is the other common choice
// @Disabled
public class Test_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftFrontMotor = null;
    DcMotor leftBackMotor = null;
    DcMotor rightFrontMotor = null;
    DcMotor rightBackMotor = null;
    DcMotor armMotor = null;
    DcMotor IntakeMotor = null;
    GyroSensor Gyro = null;
    ColorSensor LeftColorSensor = null;
    ColorSensor RightColorSensor = null;
    LightSensor LeftLightSensor = null;
    LightSensor RightLightSensor = null;

    private void Drive(double[] PowerArray){
        leftFrontMotor.setPower(PowerArray[0]);
        leftBackMotor.setPower(PowerArray[1]);
        rightFrontMotor.setPower(PowerArray[2]);
        rightBackMotor.setPower(PowerArray[3]);
    };
    private void ShiftLeft(double[] PowerArray){
        leftFrontMotor.setPower(PowerArray[0]);
        leftBackMotor.setPower(-PowerArray[1]);
        rightFrontMotor.setPower(-PowerArray[2]);
        rightBackMotor.setPower(PowerArray[3]);
    };
    private void ShiftRight(double[] PowerArray){
        leftFrontMotor.setPower(-PowerArray[0]);
        leftBackMotor.setPower(PowerArray[1]);
        rightFrontMotor.setPower(PowerArray[2]);
        rightBackMotor.setPower(-PowerArray[3]);
    };
    private boolean HitWhiteLine(ColorSensor colorSensor){
        float hsvValues[] = {0F,0F,0F};
        Color.RGBToHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), hsvValues);
        if (hsvValues[2]>0.94){return true;}
        else{return false;}
    }


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Register the Gamepad","Hit <Start> plus <A> for player 1");
        telemetry.addData("Instruction", "Left Stick: left motor; Right Stick: right motor; LT & RT: arm up/down");
        //Initialize driving motors
        try{
            leftFrontMotor = hardwareMap.dcMotor.get("left front motor");
            leftBackMotor = hardwareMap.dcMotor.get("left back motor");
            rightFrontMotor = hardwareMap.dcMotor.get("right front motor");
            rightBackMotor = hardwareMap.dcMotor.get("right back motor");
            //Set direction
            leftFrontMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
            leftBackMotor.setDirection(DcMotor.Direction.FORWARD);
            rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
            rightBackMotor.setDirection(DcMotor.Direction.REVERSE);
        }
        catch (Exception E){
            telemetry.addData("Error", E.getMessage());
            throw new InterruptedException();
        };
        //Initialize arm motor and intake motor
        boolean ArmIntakeMotorInitialized = false;
        try{
            armMotor = hardwareMap.dcMotor.get("arm motor");
            IntakeMotor = hardwareMap.dcMotor.get("intake motor");
            ArmIntakeMotorInitialized = true;
        }
        catch (Exception E){
            telemetry.addData("Warning", "Unable to access Intake motor or Arm motor!");
            telemetry.addData("Error Message",E.getMessage());
        };
        //Initialize Gyro
        boolean GyroInitialized = false;
        try{
            Gyro = hardwareMap.gyroSensor.get("Gyro");
            Gyro.calibrate();
            TimeUnit.MILLISECONDS.sleep(1500);
            GyroInitialized = true;
        }
        catch (Exception E){
            telemetry.addData("Warning", "Unable to access Gyro");
            telemetry.addData("Error Message",E.getMessage());
        };
        //Initialize Color sensor
        boolean ColorSensorInitialized=false;
        try{
            LeftColorSensor = hardwareMap.colorSensor.get("left color sensor");
            RightColorSensor = hardwareMap.colorSensor.get("right color sensor");
            //Test Color sensors if connect
            LeftColorSensor.enableLed(true);
            RightColorSensor.enableLed(true);
            TimeUnit.MILLISECONDS.sleep(1000);
            LeftColorSensor.enableLed(false);
            RightColorSensor.enableLed(false);
            ColorSensorInitialized=true;
        }
        catch (Exception E){
            telemetry.addData("Warning", "Unable to access Color sensor, Beacon Assistant is not enable!");
            telemetry.addData("Error Message",E.getMessage());
        }
        //Initialize Light sensor
        boolean LightSensorInitialized=false;
        try{
            LeftLightSensor = hardwareMap.lightSensor.get("left light sensor");
            RightLightSensor = hardwareMap.lightSensor.get("right light sensor");
            //Test Light sensors if connect
            LeftLightSensor.enableLed(true);
            RightLightSensor.enableLed(true);
            TimeUnit.MILLISECONDS.sleep(1000);
            LeftLightSensor.enableLed(false);
            RightLightSensor.enableLed(false);
            LightSensorInitialized=true;
        }
        catch (Exception E){
            telemetry.addData("Warning", "Unable to access Light sensor, Beacon Assistant is not enable!");
            telemetry.addData("Error Message",E.getMessage());
        };

        double[] PowerArray = {
                0.0,  //Left front
                0.0,  //Left back
                0.0,  //Right front
                0.0   //Right back
        };

        boolean Move = false;
        boolean IntakeMotorOn=false;
        boolean BeaconAssistantSystem = false;
        int ApproachDirection = 0;
        boolean EmergencyQuit = false;
        double EmergencyBreakTime = 0.0;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //Arm power
            if (gamepad1.right_trigger > 0) {
                armMotor.setPower(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0) {
                armMotor.setPower(-gamepad1.left_trigger);
            } else {
                armMotor.setPower(0);
            }
            //Intake motor power
            if (gamepad1.a){
                if (ArmIntakeMotorInitialized){
                    if (!IntakeMotorOn){
                        IntakeMotor.setPower(1);
                        IntakeMotorOn=true;
                        TimeUnit.MILLISECONDS.sleep(500);
                    }
                    else if (IntakeMotorOn){
                        IntakeMotor.setPower(0);
                        IntakeMotorOn=false;
                        TimeUnit.MILLISECONDS.sleep(500);
                    }
                }
            }
            else if (gamepad1.b){
                if (ArmIntakeMotorInitialized) {
                    if (!IntakeMotorOn) {
                        IntakeMotor.setPower(-1);
                        IntakeMotorOn = true;
                        TimeUnit.MILLISECONDS.sleep(500);
                    } else if (IntakeMotorOn) {
                        IntakeMotor.setPower(0);
                        IntakeMotorOn = false;
                        TimeUnit.MILLISECONDS.sleep(500);
                    }
                }
            }
            if (gamepad1.x){
                if (ColorSensorInitialized&&LightSensorInitialized){
                    if (BeaconAssistantSystem){
                        LeftColorSensor.enableLed(false);
                        RightColorSensor.enableLed(false);
                        LeftLightSensor.enableLed(false);
                        RightLightSensor.enableLed(false);
                        BeaconAssistantSystem = false;
                        TimeUnit.MILLISECONDS.sleep(500);
                    }
                    else if (!BeaconAssistantSystem){
                        LeftColorSensor.enableLed(true);
                        RightColorSensor.enableLed(true);
                        LeftLightSensor.enableLed(true);
                        RightLightSensor.enableLed(true);
                        BeaconAssistantSystem = true;
                        TimeUnit.MILLISECONDS.sleep(500);
                    }
                }
            }

            if (-gamepad1.left_stick_y!=0||-gamepad1.right_stick_y!=0){
                if (!BeaconAssistantSystem) {
                    PowerArray[0] = -gamepad1.left_stick_y;
                    PowerArray[1] = -gamepad1.left_stick_y;
                    PowerArray[2] = -gamepad1.right_stick_y;
                    PowerArray[3] = -gamepad1.right_stick_y;
                    Drive(PowerArray);
                    Move = true;
                }
                else if (BeaconAssistantSystem){
                    //Need to add a escape method!!!
                    if (EmergencyQuit&&runtime.seconds()>EmergencyBreakTime){
                        EmergencyBreakTime = 0;
                        EmergencyQuit = false;
                        telemetry.addData("Beacon Assiatant Sys","ReActivated!");
                    }
                    else if (EmergencyQuit&&runtime.seconds()<EmergencyBreakTime){
                        PowerArray[0] = -gamepad1.left_stick_y;
                        PowerArray[1] = -gamepad1.left_stick_y;
                        PowerArray[2] = -gamepad1.right_stick_y;
                        PowerArray[3] = -gamepad1.right_stick_y;
                        Drive(PowerArray);
                        Move = true;
                    }
                    else if (!EmergencyQuit) {
                        if (HitWhiteLine(LeftColorSensor)) {
                            //Ask for approach direction
                            telemetry.addData("Beacon Assistant Sys", "Please select approach direction");
                            while (true) {
                                if (gamepad1.dpad_left) {
                                    ApproachDirection = 1;
                                    break;
                                } else if (gamepad1.dpad_right) {
                                    ApproachDirection = 2;
                                    break;
                                }
                            }

                            while (true) {
                                if (!gamepad1.atRest() || gamepad1.x) {
                                    //Escape loop
                                    for (int i = 0; i <= 3; i++) {
                                        PowerArray[i] = 0.0;
                                    }
                                    Drive(PowerArray);
                                    Move = false;
                                    EmergencyQuit = true;
                                    EmergencyBreakTime = runtime.seconds() + 2;
                                    telemetry.addData("Beacon Assiatant Sys", "Aborted!");
                                    break;
                                }
                                if (HitWhiteLine(LeftColorSensor) && LeftLightSensor.getLightDetected() < 0.9) {
                                    if (ApproachDirection == 1) {
                                        PowerArray[0] = 0.05;
                                        PowerArray[1] = 0.10;
                                        PowerArray[2] = 0.20;
                                        PowerArray[3] = 0.20;
                                        Drive(PowerArray);
                                        Move = true;
                                    } else if (ApproachDirection == 2) {
                                        PowerArray[0] = 0.1;
                                        PowerArray[1] = 0.1;
                                        PowerArray[2] = 0.05;
                                        PowerArray[3] = 0.05;
                                        Drive(PowerArray);
                                        Move = true;
                                    }
                                } else if (!HitWhiteLine(LeftColorSensor) && LeftLightSensor.getLightDetected() > 0.9) {
                                    if (ApproachDirection == 1) {
                                        PowerArray[0] = -0.05;
                                        PowerArray[1] = -0.05;
                                        PowerArray[2] = 0.1;
                                        PowerArray[3] = 0.1;
                                        Drive(PowerArray);
                                        Move = true;
                                        ApproachDirection = 2; //Robot on right side
                                    } else if (ApproachDirection == 2) {
                                        PowerArray[0] = -0.05;
                                        PowerArray[1] = -0.05;
                                        PowerArray[2] = -0.1;
                                        PowerArray[3] = -0.1;
                                        Drive(PowerArray);
                                        Move = true;
                                        ApproachDirection = 1; //Robot on left side
                                    }
                                } else if (HitWhiteLine(LeftColorSensor) && LeftLightSensor.getLightDetected() > 0.9) {
                                    //Go straight
                                    for (int i = 0; i <= 3; i++) {
                                        PowerArray[i] = 0.35;
                                    }
                                    Drive(PowerArray);
                                    Move = true;
                                } else {
                                    //Situation that Robot rush out of the track during adjusting
                                    continue;
                                }
                            }
                        }
                        else if (HitWhiteLine(RightColorSensor)) {
                            //Ask for approach direction
                            telemetry.addData("Beacon Assistant Sys", "Please select approach direction");
                            while (true) {
                                if (gamepad1.dpad_left) {
                                    ApproachDirection = 1;
                                    break;
                                } else if (gamepad1.dpad_right) {
                                    ApproachDirection = 2;
                                    break;
                                }
                            }

                            while (true) {
                                if (!gamepad1.atRest() || gamepad1.x) {
                                    //Escape loop
                                    for (int i = 0; i <= 3; i++) {
                                        PowerArray[i] = 0.0;
                                    }
                                    Drive(PowerArray);
                                    Move = false;
                                    EmergencyQuit = true;
                                    EmergencyBreakTime = runtime.seconds() + 2;
                                    telemetry.addData("Beacon Assiatant Sys", "Aborted!");
                                    break;
                                }
                                if (HitWhiteLine(RightColorSensor) && RightLightSensor.getLightDetected() < 0.9) {
                                    if (ApproachDirection == 1) {
                                        PowerArray[0] = 0.05;
                                        PowerArray[1] = 0.10;
                                        PowerArray[2] = 0.20;
                                        PowerArray[3] = 0.20;
                                        Drive(PowerArray);
                                        Move = true;
                                    } else if (ApproachDirection == 2) {
                                        PowerArray[0] = 0.1;
                                        PowerArray[1] = 0.1;
                                        PowerArray[2] = 0.05;
                                        PowerArray[3] = 0.05;
                                        Drive(PowerArray);
                                        Move = true;
                                    }
                                } else if (!HitWhiteLine(RightColorSensor) && RightLightSensor.getLightDetected() > 0.9) {
                                    if (ApproachDirection == 1) {
                                        PowerArray[0] = -0.05;
                                        PowerArray[1] = -0.05;
                                        PowerArray[2] = 0.1;
                                        PowerArray[3] = 0.1;
                                        Drive(PowerArray);
                                        Move = true;
                                        ApproachDirection = 2; //Robot on right side
                                    } else if (ApproachDirection == 2) {
                                        PowerArray[0] = -0.05;
                                        PowerArray[1] = -0.05;
                                        PowerArray[2] = -0.1;
                                        PowerArray[3] = -0.1;
                                        Drive(PowerArray);
                                        Move = true;
                                        ApproachDirection = 1; //Robot on left side
                                    }
                                } else if (HitWhiteLine(RightColorSensor) && RightLightSensor.getLightDetected() > 0.9) {
                                    //Go straight
                                    for (int i = 0; i <= 3; i++) {
                                        PowerArray[i] = 0.35;
                                    }
                                    Drive(PowerArray);
                                    Move = true;
                                } else {
                                    //Situation that Robot rush out of the track during adjusting
                                    continue;
                                }
                            }
                        }
                    }
                }
            }
            else if (gamepad1.dpad_left){
                for (int i=0;i<=3;i++){PowerArray[i] = 1.0;}
                //Shifting left
                ShiftLeft(PowerArray);
                Move = true;
            }
            else if (gamepad1.dpad_right){
                for (int i=0;i<=3;i++){PowerArray[i] = 1.0;}
                //Shifting Right
                ShiftRight(PowerArray);
                Move = true;
            }
            else if(Move){
                for (int i=0;i<=3;i++){PowerArray[i] = 0.0;}
                Drive(PowerArray);
                Move = false;
            };

            telemetry.addData("Instructor", "switch drive mode: X (experimental); LT & RT: arm up/down");
            telemetry.addData("Left Stick",String.valueOf(-gamepad1.left_stick_y));
            telemetry.addData("Right Stick",String.valueOf(-gamepad1.right_stick_y));
            telemetry.addData("Left Motor Speed",String.valueOf(PowerArray[0]*100)+"%");
            telemetry.addData("Right Motor Speed",String.valueOf(PowerArray[2]*100)+"%");

            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}
