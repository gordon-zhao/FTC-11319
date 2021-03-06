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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Blue side", group="Test")  // @Red_side(...) is the other common choice
public class Blue_side extends LinearOpMode {

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
    UltrasonicSensor FrontDistanceSensor = null;

    private void Drive(double[] Powerlist){
        leftFrontMotor.setPower(Powerlist[0]);
        leftBackMotor.setPower(Powerlist[0]);
        rightFrontMotor.setPower(Powerlist[1]);
        rightBackMotor.setPower(Powerlist[1]);
    };

    @Override
    public void runOpMode() throws InterruptedException {
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
            telemetry.addData("Motors","Initialized");
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
            telemetry.addData("Shooter","Initialized");
        }
        catch (Exception E){
            telemetry.addData("Warning", "Unable to access Intake motor or Arm motor!");
            telemetry.addData("Error Message",E.getMessage());
        };
        //Initialize gyro
        boolean GyroInitialized = false;
        try{
            Gyro = hardwareMap.gyroSensor.get("gyro");
            Gyro.calibrate();
            TimeUnit.MILLISECONDS.sleep(1500);
            GyroInitialized = true;
            telemetry.addData("gyro sensor","Initialized");
        }
        catch (Exception E){
            telemetry.addData("Warning", "Unable to access gyro");
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
            telemetry.addData("Color sensors","Initialized");
        }
        catch (Exception E){
            telemetry.addData("Warning", "Unable to access Color sensor!");
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
            telemetry.addData("Light sensors","Initialized");
        }
        catch (Exception E){
            telemetry.addData("Warning", "Unable to access Light sensor!");
            telemetry.addData("Error Message",E.getMessage());
        };
        //Initialize Ultrasonic distance sensor
        boolean UltrasonicSensorInitialized = false;
        try{
            FrontDistanceSensor = hardwareMap.ultrasonicSensor.get("distance sensor");
            if (FrontDistanceSensor.getUltrasonicLevel()>=0.0){
                UltrasonicSensorInitialized = true;
                telemetry.addData("Ultrasonic sensor","Initialized");
            }
        }
        catch (Exception E){
            telemetry.addData("Warning", "Unable to access Ultrasonic sensor!");
            telemetry.addData("Error Message",E.getMessage());
        }
        double[] PowerArray = {
                0.0,  //Left front
                0.0,  //Left back
                0.0,  //Right front
                0.0   //Right back
        };
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        //Yoooooo!!!
        waitForStart();
        runtime.reset();
        //Task 1: Beacon
        boolean Task1Complete = false;
        if (ColorSensorInitialized&&LightSensorInitialized){
            if (UltrasonicSensorInitialized){
                //Go do the beacon with ultrasonic sensor
                Task1Complete = true;
            }
            else if (!UltrasonicSensorInitialized){
                if (GyroInitialized){
                    //Go do the beacon with gyro without ultrasonic sensor
                    Task1Complete = true;
                }
                else if (!GyroInitialized){
                    //Go to the beacon in blind mode
                    //If cannot hit the white line, give up
                }
            }
        };
        //Task 2: Hit the ball
        if (Task1Complete){
            //Find the blue/red line, and hit the ball
        }
        else if (!Task1Complete){
            if (ColorSensorInitialized&&LightSensorInitialized){
                //Go straight, Find the line, And hit the ball
            }
        }
        else{
            //Blind mode: Rush straight!
        }
        idle();
    }
}
