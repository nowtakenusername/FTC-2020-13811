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

@TeleOp(name="gamepad", group="Linear Opmode")
public class gamepad extends LinearOpMode {

    //Declaring OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor craneExtend = null;
    private DcMotor cranePitch = null;
    private Servo craneGrab = null;
    private Servo trayGrab = null;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Initalize the motors, servos, to their ports
        leftBackDrive  = hardwareMap.get(DcMotor.class, "leftBackDrive"); //port 0, hub1
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive"); //port 1, hub1
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive"); //port 2, hub1
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive"); //port 3, hub1
        craneExtend = hardwareMap.get(DcMotor.class, "craneExtend"); //port 0, hub2
        cranePitch = hardwareMap.get(DcMotor.class, "cranePitch"); //port 1, hub2
        craneGrab = hardwareMap.get(Servo.class, "craneGrab"); //port 0, servo
        trayGrab = hardwareMap.get(Servo.class, "trayGrab"); //port 1, servo

        //Setting the motor directions
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);

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
            double triggerPowerRight2;
            double triggerPowerLeft2;
            double craneExtendPower;
            double cranePitchPower;
            double craneGrabPos = 0;
            double trayGrabPos = 0;


            //the below variables are containing the value of the gamepad joysticks and other buttons as they are being pressed/moved through each loop
            //these are the variables for the left and right velocities/powers set for each wheel
            leftPower  = gamepad1.left_stick_y;
            rightPower = gamepad1.right_stick_y;
            //these are the variables for the crab-like movement of the mechanum wheel robot
            moveLeftPower = -gamepad1.left_stick_x;
            moveRightPower = -gamepad1.left_stick_x;
            //these are the variables for the triggers on the gamepad to switch between the "crab" mode to the regular "tank" mode
            triggerPowerLeft = gamepad1.left_trigger;
            triggerPowerRight = gamepad1.right_trigger;
            //these are servo variables
            triggerPowerLeft2 = gamepad2.left_trigger;
            triggerPowerRight2 = gamepad2.right_trigger;
            //this is the power variable for the crane
            craneExtendPower = gamepad2.left_stick_y;
            cranePitchPower = gamepad2.right_stick_y;


            // Send calculated power to wheels
            //if both of the triggers on the gamepad are not being pulled, then send the calculated velocities/powers to the wheels to replicate "tank" mode for the robot
            if (triggerPowerLeft == 0 && triggerPowerRight == 0) {
                leftBackDrive.setPower(leftPower);
                rightBackDrive.setPower(rightPower);
                leftFrontDrive.setPower(leftPower);
                rightFrontDrive.setPower(rightPower);
            }

            //else if one of the triggers are being pushed (meaning "crab" mode), then send the calculated velocities/powers to the wheels to replicate "crab" mode for the robot
            else if (triggerPowerLeft > 0 || triggerPowerRight > 0) {
                leftBackDrive.setPower(moveLeftPower);
                rightBackDrive.setPower(moveRightPower);
                leftFrontDrive.setPower(-moveLeftPower);
                rightFrontDrive.setPower(-moveRightPower);
            }

            //this is the part of the code that deals with the crane
            //craneExtend.setPower(craneExtendPower);
            //cranePitch.setPower(cranePitchPower);

            if(triggerPowerRight2==0) { //Tray grabber (goes 0 to 20 (actually 18) )
                trayGrabPos = 0;
                trayGrab.setPosition(trayGrabPos);
            }
            else if(triggerPowerRight2>0) {
                trayGrabPos = 0.1;
                trayGrab.setPosition(trayGrabPos);
            }

            if(triggerPowerLeft2==0) { //Crane grabber (goes 0 to 90)
                craneGrabPos = 0;
                craneGrab.setPosition(craneGrabPos);
            }
            else if(triggerPowerLeft2>0) {
                craneGrabPos = 0.6;
                craneGrab.setPosition(craneGrabPos);
            }

            // Shows the elapsed game time and other data on the android device.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("CrabMotors", "left (%.2f), right (%.2f)", moveLeftPower, moveRightPower);
            telemetry.addData("Triggers", "left (%.2f), right (%.2f)", triggerPowerLeft, triggerPowerRight);
            telemetry.addData("Crane Motors", "craneExtend (%.2f), cranePitch (%.2f)", craneExtend, cranePitch);
            telemetry.addData("Grabbers", "craneGrab (%.2f), trayGrab (%.2f)", craneGrabPos, trayGrabPos);
            telemetry.update();
        }
    }
}
