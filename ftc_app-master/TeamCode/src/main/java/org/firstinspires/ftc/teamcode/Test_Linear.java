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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.LegacyModule;
import com.qualcomm.robotcore.hardware.GyroSensor;

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


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Regist the Gamepad","Hit <Start> plus <A> for player 1");
        telemetry.addData("Instruction", "Left Stick: left motor; Right Stick: right motor; LT & RT: arm up/down");
        //Initialize the motors
        try{
            leftFrontMotor = hardwareMap.dcMotor.get("left front motor");
            leftBackMotor = hardwareMap.dcMotor.get("left back motor");
            rightFrontMotor = hardwareMap.dcMotor.get("right front motor");
            rightBackMotor = hardwareMap.dcMotor.get("right back motor");
        }
        catch (Exception E){
            telemetry.addData("Error", E.getMessage());
            throw new InterruptedException();
        };
        
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

        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        leftBackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);

        double[] PowerArray = {
                0.0,  //Left front
                0.0,  //Left back
                0.0,  //Right front
                0.0   //Right back
        };

        boolean Move = false;
        boolean IntakeMotorOn=false;

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
            if (-gamepad1.left_stick_y!=0||-gamepad1.right_stick_y!=0){
                PowerArray[0] = -gamepad1.left_stick_y;
                PowerArray[1] = -gamepad1.left_stick_y;
                PowerArray[2] = -gamepad1.right_stick_y;
                PowerArray[3] = -gamepad1.right_stick_y;
                Drive(PowerArray);
                Move = true;
            }
            else if (gamepad1.dpad_left){
                for (int i=0;i<=3;i++){
                    PowerArray[i] = 1.0;
                }
                //Shifting left
                ShiftLeft(PowerArray);
                Move = true;
            }
            else if (gamepad1.dpad_right){
                for (int i=0;i<=3;i++){
                    PowerArray[i] = 1.0;
                }
                //Shifting Right
                ShiftRight(PowerArray);
                Move = true;
            }
            else if(Move){
                for (int i=0;i<=3;i++){
                    PowerArray[i] = 0.0;
                }
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
