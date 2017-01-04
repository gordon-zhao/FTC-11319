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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "Manual", group = "Test")  // @Red_side(...) is the other common choice
// @Disabled
public class TeleOP extends LinearOpMode {

    // Motors
    protected DcMotor leftFrontMotor = null;
    protected DcMotor leftBackMotor = null;
    protected DcMotor rightFrontMotor = null;
    protected DcMotor rightBackMotor = null;
    protected DcMotor armMotor = null;
    protected DcMotor intakeMotor = null;

    // Gyro
    protected GyroSensor gyro = null;

    // Sensors
    // color sensor
    protected ColorSensor leftColorSensor = null;
    protected ColorSensor rightColorSensor = null;
    // light sensor
    protected LightSensor leftLightSensor = null;
    protected LightSensor rightLightSensor = null;

    // members to flush console screen with
    private Map<String, String> printOut = new HashMap<>();
    // CPU time
    private ElapsedTime runtime = new ElapsedTime();

    private void drive(double[] power) {
        leftFrontMotor.setPower(power[0]);
        leftBackMotor.setPower(power[1]);
        rightFrontMotor.setPower(power[2]);
        rightBackMotor.setPower(power[3]);
    }

    private void shiftLeft(double[] power) {
        leftFrontMotor.setPower(power[0]);
        leftBackMotor.setPower(-power[1]);
        rightFrontMotor.setPower(-power[2]);
        rightBackMotor.setPower(power[3]);
    }

    private void shiftRight(double[] power) {
        leftFrontMotor.setPower(-power[0]);
        leftBackMotor.setPower(power[1]);
        rightFrontMotor.setPower(power[2]);
        rightBackMotor.setPower(-power[3]);
    }

    private boolean hitWhiteLine(ColorSensor colorSensor) {
        float hsvValues[] = {0F, 0F, 0F};
        Color.RGBToHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), hsvValues);
        return hsvValues[2] > 0.94;
    }

    private void print(Map<String, String> context) {
        for (String key : context.keySet()) {
            telemetry.addData(key, context.get(key));
        }
        telemetry.update();
    }

    private boolean initArmMotor() {
        try {
            armMotor = hardwareMap.dcMotor.get("arm motor");
            intakeMotor = hardwareMap.dcMotor.get("intake motor");
            telemetry.addData("Shooter", "Initialized");
            return true;
        } catch (Exception e) {
            telemetry.addData("Error", "Unable to access Intake motor or Arm motor!");
            telemetry.addData("Error Message", e.getMessage());
            return false;
        }
    }

    private boolean initgyro() {
        try {
            gyro = hardwareMap.gyroSensor.get("gyro");
            gyro.calibrate();
            TimeUnit.MILLISECONDS.sleep(1500);
            telemetry.addData("gyro sensor", "Initialized");
            return true;
        } catch (Exception e) {
            telemetry.addData("Warning", "Unable to access gyro");
            telemetry.addData("Error Message", e.getMessage());
            return false;
        }
    }

    private boolean initColorSensor() {
        try {
            leftColorSensor = hardwareMap.colorSensor.get("left color sensor");
            rightColorSensor = hardwareMap.colorSensor.get("right color sensor");
            //Test Color sensors if connect
            leftColorSensor.enableLed(true);
            rightColorSensor.enableLed(true);
            TimeUnit.MILLISECONDS.sleep(1000);
            leftColorSensor.enableLed(false);
            rightColorSensor.enableLed(false);
            telemetry.addData("Color sensors", "Initialized");
            return true;
        } catch (Exception e) {
            telemetry.addData("Warning", "Unable to access Color sensor, Beacon Assistant is not enable!");
            telemetry.addData("Error Message", e.getMessage());
            return false;
        }

    }

    private boolean initLightSensor() {
        try {
            leftLightSensor = hardwareMap.lightSensor.get("left light sensor");
            rightLightSensor = hardwareMap.lightSensor.get("right light sensor");
            //Test Light sensors if connect
            leftLightSensor.enableLed(true);
            rightLightSensor.enableLed(true);
            TimeUnit.MILLISECONDS.sleep(1000);
            leftLightSensor.enableLed(false);
            rightLightSensor.enableLed(false);
            telemetry.addData("Light sensors", "Initialized");
            return true;
        } catch (Exception e) {
            telemetry.addData("Warning", "Unable to access Light sensor, Beacon Assistant is not enable!");
            telemetry.addData("Error Message", e.getMessage());
            return false;
        }
    }

    void sleep(int ms) {
        try {
            TimeUnit.MILLISECONDS.sleep(ms);
        } catch (InterruptedException ie) {
            // TODO
        }
    }

    private void runIntakeMotor(boolean isArmIntake, boolean isIntakeMotor) {

        //Intake motor power
        if (gamepad1.a) {
            if (isArmIntake) {
                if (!isIntakeMotor) {
                    intakeMotor.setPower(1);
                    isIntakeMotor = true;
                    sleep(500);
                } else if (isIntakeMotor) {
                    intakeMotor.setPower(0);
                    isIntakeMotor = false;
                    sleep(500);
                }
            }
        } else if (gamepad1.b) {
            if (isArmIntake) {
                if (!isIntakeMotor) {
                    intakeMotor.setPower(-1);
                    isIntakeMotor = true;
                    sleep(500);
                } else if (isIntakeMotor) {
                    intakeMotor.setPower(0);
                    isIntakeMotor = false;
                    sleep(500);
                }
            }
        }
    }

    private void runLightSensors(boolean isColorSensor, boolean isLightSensor, boolean beaconAssistantSystem) {

        if (gamepad1.x) {
            if (isColorSensor && isLightSensor) {
                if (beaconAssistantSystem) {
                    leftColorSensor.enableLed(false);
                    rightColorSensor.enableLed(false);
                    leftLightSensor.enableLed(false);
                    rightLightSensor.enableLed(false);
                    beaconAssistantSystem = false;
                    printOut.put("Beacon Assistant Sys", "Off");
                    print(printOut);
                    sleep(500);
                } else if (!beaconAssistantSystem) {
                    leftColorSensor.enableLed(true);
                    rightColorSensor.enableLed(true);
                    leftLightSensor.enableLed(true);
                    rightLightSensor.enableLed(true);
                    beaconAssistantSystem = true;
                    printOut.put("Beacon Assistant Sys", "On");
                    print(printOut);
                    sleep(500);
                }
            }
        }

    }

    @Override
    public void runOpMode() throws InterruptedException {
        printOut.put("Register the Gamepad", "Hit <Start> plus <A> for player 1");
        printOut.put("Instruction", "Left Stick: left motor; Right Stick: right motor; LT & RT: arm up/down");
        print(printOut);
        //Initialize driving motors
        leftFrontMotor = hardwareMap.dcMotor.get("left front motor");
        leftBackMotor = hardwareMap.dcMotor.get("left back motor");
        rightFrontMotor = hardwareMap.dcMotor.get("right front motor");
        rightBackMotor = hardwareMap.dcMotor.get("right back motor");
        //Set direction
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        leftBackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("Motors", "Initialized");
        //Initialize arm motor and intake motor
        boolean isArmIntake = initArmMotor();
        //Initialize gyro
        boolean isGyro = initgyro();
        //Initialize Color sensor
        boolean isColorSensor = initColorSensor();
        //Initialize Light sensor
        boolean isLightSensor = initLightSensor();

        double[] power = {
                0.0,  //Left front
                0.0,  //Left back
                0.0,  //Right front
                0.0   //Right back
        };

        boolean isMoving = false;
        boolean isIntakeMotor = false;
        boolean beaconAssistantSystem = false;
        int approachDirection = 0;
        boolean emergencyQuit = false;
        double emergencyBreakTime = 0.0;

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        runtime.reset();
        // run until the end of the match (until driver presses STOP)
        while (opModeIsActive()) {
            printOut.put("Status", "Run Time: " + runtime.toString());
            printOut.put("Instructor", "switch drive mode: X (experimental); LT & RT: arm up/down");
            printOut.put("Left Stick", String.valueOf(-gamepad1.left_stick_y));
            printOut.put("Right Stick", String.valueOf(-gamepad1.right_stick_y));
            printOut.put("Left Motor Speed", String.valueOf(power[0] * 100) + "%");
            printOut.put("Right Motor Speed", String.valueOf(power[2] * 100) + "%");
            print(printOut);
            armMotor.setPower((gamepad1.right_trigger > 0) ? (gamepad1.right_trigger) :
                    (- gamepad1.left_trigger));

            //Intake motor power
            runIntakeMotor(isArmIntake, isIntakeMotor);
            // Light sensor power
            runLightSensors(isColorSensor, isLightSensor, beaconAssistantSystem);

            if (-gamepad1.left_stick_y != 0 || -gamepad1.right_stick_y != 0) {
                if (!beaconAssistantSystem) {
                    power[0] = gamepad1.left_stick_y / 2;
                    power[1] = gamepad1.left_stick_y / 2;
                    power[2] = gamepad1.right_stick_y / 2;
                    power[3] = gamepad1.right_stick_y / 2;
                    drive(power);
                    isMoving = true;
                } else if (beaconAssistantSystem) {
                    //Need to add a escape method!!!
                    if (emergencyQuit && runtime.seconds() > emergencyBreakTime) {
                        emergencyBreakTime = 0;
                        emergencyQuit = false;
                        printOut.put("Beacon Assiatant Sys", "ReActivated!");
                        print(printOut);
                    } else if (emergencyQuit && runtime.seconds() < emergencyBreakTime) {
                        power[0] = gamepad1.left_stick_y / 2;
                        power[1] = gamepad1.left_stick_y / 2;
                        power[2] = gamepad1.right_stick_y / 2;
                        power[3] = gamepad1.right_stick_y / 2;
                        drive(power);
                        isMoving = true;
                    } else if (!emergencyQuit) {
                        if (hitWhiteLine(leftColorSensor)) {
                            //Ask for approach direction
                            printOut.put("Beacon Assistant Sys", "Please select approach direction");
                            print(printOut);
                            while (true) {
                                if (gamepad1.dpad_left) {
                                    approachDirection = 1;
                                    break;
                                } else if (gamepad1.dpad_right) {
                                    approachDirection = 2;
                                    break;
                                }
                            }

                            while (true) {
                                if (!gamepad1.atRest() || gamepad1.x) {
                                    //Escape loop
                                    for (int i = 0; i <= 3; i++) {
                                        power[i] = 0.0;
                                    }
                                    drive(power);
                                    isMoving = false;
                                    emergencyQuit = true;
                                    emergencyBreakTime = runtime.seconds() + 2;
                                    printOut.put("Beacon Assiatant Sys", "Aborted!");
                                    print(printOut);
                                    break;
                                }
                                if (hitWhiteLine(leftColorSensor) && leftLightSensor.getLightDetected() < 0.9) {
                                    if (approachDirection == 1) {
                                        power[0] = 0.05;
                                        power[1] = 0.10;
                                        power[2] = 0.20;
                                        power[3] = 0.20;
                                        drive(power);
                                        isMoving = true;
                                    } else if (approachDirection == 2) {
                                        power[0] = 0.1;
                                        power[1] = 0.1;
                                        power[2] = 0.05;
                                        power[3] = 0.05;
                                        drive(power);
                                        isMoving = true;
                                    }
                                } else if (!hitWhiteLine(leftColorSensor) && leftLightSensor.getLightDetected() > 0.9) {
                                    if (approachDirection == 1) {
                                        power[0] = -0.05;
                                        power[1] = -0.05;
                                        power[2] = 0.1;
                                        power[3] = 0.1;
                                        drive(power);
                                        isMoving = true;
                                        approachDirection = 2; //Robot on right side
                                    } else if (approachDirection == 2) {
                                        power[0] = -0.05;
                                        power[1] = -0.05;
                                        power[2] = -0.1;
                                        power[3] = -0.1;
                                        drive(power);
                                        isMoving = true;
                                        approachDirection = 1; //Robot on left side
                                    }
                                } else if (hitWhiteLine(leftColorSensor) && leftLightSensor.getLightDetected() > 0.9) {
                                    //Go straight
                                    for (int i = 0; i <= 3; i++) {
                                        power[i] = 0.35;
                                    }
                                    drive(power);
                                    isMoving = true;
                                } else {
                                    //Situation that Robot rush out of the track during adjusting
                                    //Now just head back and redo
                                    for (int i = 0; i <= 3; i++) {
                                        power[i] = -power[i];
                                    }
                                    TimeUnit.MILLISECONDS.sleep(500);
                                }
                            }
                        } else if (hitWhiteLine(rightColorSensor)) {
                            //Ask for approach direction
                            printOut.put("Beacon Assistant Sys", "Please select approach direction");
                            print(printOut);
                            while (true) {
                                if (gamepad1.dpad_left) {
                                    approachDirection = 1;
                                    break;
                                } else if (gamepad1.dpad_right) {
                                    approachDirection = 2;
                                    break;
                                }
                            }

                            while (true) {
                                if (!gamepad1.atRest() || gamepad1.x) {
                                    //Escape loop
                                    for (int i = 0; i <= 3; i++) {
                                        power[i] = 0.0;
                                    }
                                    drive(power);
                                    isMoving = false;
                                    emergencyQuit = true;
                                    emergencyBreakTime = runtime.seconds() + 2;
                                    printOut.put("Beacon Assiatant Sys", "Aborted!");
                                    print(printOut);
                                    break;
                                }
                                if (hitWhiteLine(rightColorSensor) && rightLightSensor.getLightDetected() < 0.9) {
                                    if (approachDirection == 1) {
                                        power[0] = 0.05;
                                        power[1] = 0.10;
                                        power[2] = 0.20;
                                        power[3] = 0.20;
                                        drive(power);
                                        isMoving = true;
                                    } else if (approachDirection == 2) {
                                        power[0] = 0.1;
                                        power[1] = 0.1;
                                        power[2] = 0.05;
                                        power[3] = 0.05;
                                        drive(power);
                                        isMoving = true;
                                    }
                                } else if (!hitWhiteLine(rightColorSensor) && rightLightSensor.getLightDetected() > 0.9) {
                                    if (approachDirection == 1) {
                                        power[0] = -0.05;
                                        power[1] = -0.05;
                                        power[2] = 0.1;
                                        power[3] = 0.1;
                                        drive(power);
                                        isMoving = true;
                                        approachDirection = 2; //Robot on right side
                                    } else if (approachDirection == 2) {
                                        power[0] = -0.05;
                                        power[1] = -0.05;
                                        power[2] = -0.1;
                                        power[3] = -0.1;
                                        drive(power);
                                        isMoving = true;
                                        approachDirection = 1; //Robot on left side
                                    }
                                } else if (hitWhiteLine(rightColorSensor) && rightLightSensor.getLightDetected() > 0.9) {
                                    //Go straight
                                    for (int i = 0; i <= 3; i++) {
                                        power[i] = 0.35;
                                    }
                                    drive(power);
                                    isMoving = true;
                                } else {
                                    //Situation that Robot rush out of the track during adjusting
                                    //Now just head back and redo
                                    for (int i = 0; i <= 3; i++) {
                                        power[i] = -power[i];
                                    }
                                    sleep(500);
                                }
                            }
                        }
                    }
                }
            } else if (gamepad1.dpad_left) {
                for (int i = 0; i <= 3; i++) {
                    power[i] = 1.0;
                }
                //Shifting left
                shiftLeft(power);
                isMoving = true;
            } else if (gamepad1.dpad_right) {
                for (int i = 0; i <= 3; i++) {
                    power[i] = 1.0;
                }
                //Shifting Right
                shiftRight(power);
                isMoving = true;
            } else if (isMoving) {
                for (int i = 0; i <= 3; i++) {
                    power[i] = 0.0;
                }
                drive(power);
                isMoving = false;
            }
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop

        }
    }
}
