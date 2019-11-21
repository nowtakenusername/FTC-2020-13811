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
import com.qualcomm.robotcore.hardware.HardwareMap; //TAKE PITY ON ME HARDWAREMAP.JAVA

@TeleOp(name="gamepad", group="Linear Opmode")
//@Disabled
public class gamepad extends LinearOpMode {

    //Declaring OpMode members
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftFrontDrive;
    private DcMotor craneExtend;
    private DcMotor cranePitch;
    private DcMotor craneElevate;
    private Servo craneGrab;
    private Servo trayGrab;

    public void runOpMode() {

        //Initalize the motors, servos, to their ports
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive"); //port 0, hub1
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive"); //port 1, hub1
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive"); //port 2, hub1
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive"); //port 3, hub1
        craneExtend = hardwareMap.get(DcMotor.class, "craneExtend"); //port 0, hub2
        cranePitch = hardwareMap.get(DcMotor.class, "cranePitch"); //port 1, hub2
        craneElevate = hardwareMap.get(DcMotor.class, "craneElevate"); //port 2, hub2
        craneGrab = hardwareMap.get(Servo.class, "craneGrab"); //port 0, servo
        trayGrab = hardwareMap.get(Servo.class, "trayGrab"); //port 1, servo

        //Setting the motor directions
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);

        craneElevate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        craneElevate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update(); //Done "initalizing"

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
            double craneElevatePower;
            double craneGrabPos = 0;
            double trayGrabPos = 0;

            //the below variables are containing the value of the gamepad joysticks and other buttons as they are being pressed/moved through each loop
            //these are the variables for the left and right wheels
            leftPower  = gamepad1.left_stick_y;
            rightPower = gamepad1.right_stick_y;
            //these are the variables for the crab-like movement of the mechanum wheels
            moveLeftPower = -gamepad1.left_stick_x;
            moveRightPower = -gamepad1.left_stick_x;
            //these are the variables for the triggers on the gamepad, if pressed the robot will go to "crab mode"
            triggerPowerLeft = gamepad1.left_trigger;
            triggerPowerRight = gamepad1.right_trigger;
            //these are the variables for the triggers on the gamepad, utilised for the crane's grasper and the tray clip
            triggerPowerLeft2 = gamepad2.left_trigger;
            triggerPowerRight2 = gamepad2.right_trigger;
            //these are the power variables for the crane
            craneExtendPower = gamepad2.left_stick_y;
            cranePitchPower = gamepad2.right_stick_y;

            // Send calculated power to wheels
            //if both of the triggers on the gamepad are not being pulled, then send the calculated velocities/powers to the wheels to replicate "tank" mode for the robot
            if (triggerPowerLeft == 0 && triggerPowerRight == 0) {
                if(gamepad1.left_stick_y>0.5) {
                    leftBackDrive.setPower(leftPower);
                    leftFrontDrive.setPower(leftPower);
                }
                else {
                    leftBackDrive.setPower(leftPower/2);
                    leftFrontDrive.setPower(leftPower/2);
                }
                if(gamepad1.right_stick_y>0.5) {
                    rightBackDrive.setPower(rightPower);
                    rightFrontDrive.setPower(rightPower);
                }
                else {
                    rightBackDrive.setPower(rightPower/2);
                    rightFrontDrive.setPower(rightPower/2);
                }

                if(gamepad1.left_bumper) //making small adjustments... 
                    leftPower = 0.1;

                if(gamepad1.right_bumper)
                    rightPower = 0.1;

            }

            //else if one of the triggers are being pushed (meaning "crab" mode), then send the calculated velocities/powers to the wheels to replicate "crab" mode for the robot
            else if (triggerPowerLeft > 0 || triggerPowerRight > 0) {
                leftBackDrive.setPower(moveLeftPower);
                rightBackDrive.setPower(-moveRightPower);
                leftFrontDrive.setPower(-moveLeftPower);
                rightFrontDrive.setPower(moveRightPower);
            }

            //this is the part of the code that deals with the crane
            if(gamepad2.y) { //press y, it goes up. press a, it goes down. it's so much harder than it sounds
                craneElevate.setTargetPosition(2000);
                while(craneElevate.getCurrentPosition()<2000)
                    cranePitch.setPower(0.35); //makes the crane match it
                craneElevate.setPower(1);
            }
            else
            {
                craneElevate.setPower(0);
            }
            if(gamepad2.a) {
                craneElevate.setTargetPosition(0);
                while(craneElevate.getCurrentPosition()>0)
                    craneElevate.setPower(-0.5);
                cranePitch.setPower(0.05); //yeah, its goes down real fast
            }
            else
            {
                craneElevate.setPower(0);
            }

            if(triggerPowerRight2==0) { //Tray grabber (goes 0 to 20 (actually 18) )
                trayGrabPos = 0;
                trayGrab.setPosition(trayGrabPos);
            }
            else if(triggerPowerRight2>0) {
                trayGrabPos = 0.1;
                trayGrab.setPosition(trayGrabPos);
            }

            if(triggerPowerLeft2==0) { //Crane grabber (goes 0 to 90)
                craneGrabPos = 0.5;
                craneGrab.setPosition(craneGrabPos);
                cranePitch.setPower(-cranePitchPower);
            }
            else if(triggerPowerLeft2>0) {
                craneGrabPos = 0;
                craneGrab.setPosition(craneGrabPos);
                if(gamepad2.right_stick_y!=0) //Adds a bit of power to keep the crane stable
                {
                    cranePitch.setPower(-cranePitchPower);
                }
                else
                {
                    cranePitch.setPower(0.02);
                }
            }

            //However, for testing purposes and emergency purposes there will be a needed part to the code: 
            if(gamepad2.left_trigger==1 && gamepad2.right_trigger==1) //The Debug Menu! press both triggers to enter
            {
                //craneElevate.setPower(gamepad2.left_stick_y); //spools up the pulley if needed for emergencies
                cranePitch.setPower(-gamepad2.right_stick_y);
            }

            //Shows the elapsed game time and other data on the android device.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("CrabMotors", "left (%.2f), right (%.2f)", moveLeftPower, moveRightPower);
            telemetry.addData("Crane Motors", "craneExtend (%.2f), cranePitch (%.2f)", craneExtendPower, cranePitchPower);
            telemetry.addData("Crane Position", craneElevate.getCurrentPosition());
            telemetry.addData("Grabbers", "craneGrab (%.2f), trayGrab (%.2f)", craneGrabPos, trayGrabPos);
            telemetry.update();
        }
    }
}
