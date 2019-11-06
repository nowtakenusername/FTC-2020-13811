/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="gamepad", group="Linear Opmode")
//@Disabled
public class gamepad extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftFrontDrive = null;
    private Servo craneX = null;
    private Servo craneY = null;
    private Servo craneGrab = null;
    private Servo trayGrab = null;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //The below lines of code initialize the hardware variables to be used later on (such as the motors and servos)
        leftDrive  = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        craneX = hardwareMap.get(Servo.class, "craneX");
        craneY = hardwareMap.get(Servo.class, "craneY");
        craneGrab = hardwareMap.get(Servo.class, "craneGrab");
        trayGrab = hardwareMap.get(Servo.class, "trayGrab");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        //IMPORTANT: Port 0 = leftDrive, Port 1 = rightDrive, Port 2 = leftFrontDrive, Port 3 = rightFrontDrive... that's how the ports are labeled on the basic Overload Code config.

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;
            double moveLeftPower;
            double moveRightPower;
            double triggerPowerLeft;
            double triggerPowerRight;
            double craneXPos = 0;
            double craneYPos = 0;
            double craneGrabPos = 0;
            double trayGrabPos = 0;
            double triggerPowerRight2;


            //the below variables are containing the value of the gamepad joysticks and other buttons as they are being pressed/moved through each loop
            //these are the variables for the left and right velocities/powers set for each wheel
            leftPower  = -gamepad1.left_stick_y;
            rightPower = -gamepad1.right_stick_y;
            //these are the variables for the crab-like movement of the mechanum wheel robot
            moveLeftPower = -gamepad1.left_stick_x;
            moveRightPower = -gamepad1.left_stick_x;
            //these are the variables for the triggers on the gamepad to switch between the "crab" mode to the regular "tank" mode
            triggerPowerLeft = gamepad1.left_trigger;
            triggerPowerRight = gamepad1.right_trigger;
            //these are servo variables
            triggerPowerRight2 = gamepad2.right_trigger;


            // Send calculated power to wheels
            //if both of the triggers on the gamepad are not being pulled, then send the calculated velocities/powers to the wheels to replicate "tank" mode for the robot
            if (triggerPowerLeft == 0 && triggerPowerRight == 0) {
                leftDrive.setPower(-leftPower);
                rightDrive.setPower(-rightPower);
                leftFrontDrive.setPower(leftPower);
                rightFrontDrive.setPower(rightPower);
            }
            //else if one of the triggers are being pushed (meaning "crab" mode), then send the calculated velocities/powers to the wheels to replicate "crab" mode for the robot
            else if (triggerPowerLeft > 0 || triggerPowerRight > 0) {
                leftDrive.setPower(-moveLeftPower);
                rightDrive.setPower(-moveRightPower);
                leftFrontDrive.setPower(moveLeftPower);
                rightFrontDrive.setPower(moveRightPower);
            }
            //and here's the servos
            if(triggerPowerRight2==0)//traygrabber
            {
                trayGrabPos = 0;
                trayGrab.setPosition(trayGrabPos);
            }
            else if(triggerPowerRight2>0)
            {
                trayGrabPos = 0.1;
                trayGrab.setPosition(trayGrabPos);
            }

            // Show the elapsed game time and wheel power on the android device.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("CrabMotors", "left (%.2f), right (%.2f)", moveLeftPower, moveRightPower);
            telemetry.addData("Triggers", "left (%.2f), right (%.2f)", triggerPowerLeft, triggerPowerRight);
            telemetry.addData("Crane Servos", "craneX (%.2f), craneY (%.2f)", craneXPos, craneYPos);
            telemetry.addData("Grabbers", "craneGrab (%.2f), trayGrab (%.2f)", craneGrabPos, trayGrabPos);
            telemetry.update();
        }
    }
}
