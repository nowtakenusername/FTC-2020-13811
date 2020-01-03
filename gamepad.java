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
import com.qualcomm.robotcore.hardware.HardwareMap;

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
    private Servo craneGrabAttitude;

    public void runOpMode() {
        
//******************************************************************************

        //Initial setup
        //Initialize the motors, servos, to their ports
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive"); //port 0, hub1
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive"); //port 1, hub1
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive"); //port 2, hub1
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive"); //port 3, hub1
        craneExtend = hardwareMap.get(DcMotor.class, "craneExtend"); //port 0, hub2
        cranePitch = hardwareMap.get(DcMotor.class, "cranePitch"); //port 1, hub2
        craneElevate = hardwareMap.get(DcMotor.class, "craneElevate"); //port 2, hub2
        craneGrab = hardwareMap.get(Servo.class, "craneGrab"); //port 0, servo
        trayGrab = hardwareMap.get(Servo.class, "trayGrab"); //port 1, servo
        
        //Zeroes out encoders
        craneElevate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cranePitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        //Sets motors to run to [pulses] encoder pulses
        craneElevate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        cranePitch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        //Setting the motor directions
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);

        //Initializing some picky variables
        double runtimeWait = 0;
        int craneElevatePulses = 0;
          int craneSetting = 1; //Starting barely not scraping the ground
        //int craneSetting = 0; //Starting in compact mode

        telemetry.addData("Status", "Initialized");
        telemetry.update(); //Done initalizing

        //Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
//******************************************************************************
            
            //Variable setup
            //Setting up variables - many are redundant and are pending removal
            double leftPower; //Tank movement left wheels [pending removal]
            double rightPower; //Tank movement right wheels [pending removal]
            double moveLeftPower; //Crab movement leftwards [pending removal]
            double moveRightPower; //Crab movement rightwards [pending removal]
            double triggerPowerLeft; //Gamepad 1 left trigger power [pending removal]
            double triggerPowerRight; //Gamepad 1 right trigger power [pending removal]
            double triggerPowerRight2; //Gamepad 2 right trigger power [pending removal]
            double triggerPowerLeft2; //Gamepad 2 left trigger power [pending removal]
            double craneExtendPower; //Crane linear actuator motor power
            double cranePitchPower; //Crane pitch motor power
            double craneElevatePower; //Crane lift motor power
            double craneGrabPos = 0; //Grabber claw servo position
            double trayGrabPos = 0; //Tray grabber servo position
            double craneGrabAttitudePos = 0; //Grabber attitude servo position
            double cranePosOffset = 0; //(un)used to offset future values if starting at craneSetting 0

            //The below variables contain the value of the gamepad joysticks and other buttons as they are being pressed/moved through each loop 
            //All are repetitive and will be removed alongside other pending removals and replaced with their gamepad counterparts for simplicity's sake
            //These are the variables for the left and right wheels
            leftPower  = gamepad1.left_stick_y;
            rightPower = gamepad1.right_stick_y;
            //these are the variables for the crab-like movement of the mechanum wheels
            moveLeftPower = -gamepad1.left_stick_x;
            moveRightPower = -gamepad1.left_stick_x;
            //These are the variables for the triggers on the gamepad, if pressed the robot will go to "crab mode"
            triggerPowerLeft = gamepad1.left_trigger;
            triggerPowerRight = gamepad1.right_trigger;
            //these are the variables for the triggers on the gamepad, utilised for the crane's grasper and the tray clip
            triggerPowerLeft2 = gamepad2.left_trigger;
            triggerPowerRight2 = gamepad2.right_trigger;
            //These are the power variables for the crane
            craneExtendPower = gamepad2.left_stick_y;
            cranePitchPower = gamepad2.right_stick_y;

//******************************************************************************

            //Drive controls
            //Tank mode, the gnarly if elses allow finer tuning of robot movement
            if (gamepad1.left_trigger==0 && gamepad1.right_trigger==0) {
                if(gamepad1.left_stick_y>0.7 || gamepad1.left_stick_y<-0.7) {
                    leftBackDrive.setPower(leftPower/1.5); //Full throttle
                    leftFrontDrive.setPower(leftPower/1.5);
                }
                else {
                    leftBackDrive.setPower(leftPower/2); //Adjustment throttle
                    leftFrontDrive.setPower(leftPower/2);
                }
                if(gamepad1.right_stick_y>0.7 || gamepad1.right_stick_y<-0.7) {
                    rightBackDrive.setPower(rightPower/1.5); //Full throttle
                    rightFrontDrive.setPower(rightPower/1.5);
                }
                else {
                    rightBackDrive.setPower(rightPower/2); //Adjustment throttle
                    rightFrontDrive.setPower(rightPower/2);
                }
            }

            //Crab mode
            //As crab mode is fairly power intensive there are no limits for the time being
            else if (triggerPowerLeft>0 || triggerPowerRight>0) {
                leftBackDrive.setPower(-moveLeftPower);
                rightBackDrive.setPower(moveRightPower);
                leftFrontDrive.setPower(moveLeftPower);
                rightFrontDrive.setPower(-moveRightPower);
            }
            
//******************************************************************************

            //Crane control
            //Runs the linear actuator
            if(gamepad2.y) { //Extends the crane arm
                craneExtend.setPower(-1);
            }
            else if(gamepad2.a) {
                craneExtend.setPower(1);
            }
            else
                craneExtend.setPower(0);

            //sets the automated crane setting down by one
            if(gamepad2.dpad_down==true && runtimeWait<=runtime.seconds() && craneSetting > 0) { 
                craneSetting -= 1; 
                craneElevate.setPower(0.2);
                cranePitch.setPower(0.2);
                runtimeWait=runtime.seconds()+0.25; //Sets a timer for .25 seconds until it can go down another setting
            }
            //sets the automated crane setting up by one
            else if(gamepad2.dpad_up==true && runtimeWait<=runtime.seconds() && craneSetting < 12) { 
                craneSetting += 1; 
                craneElevate.setPower(0.5);
                cranePitch.setPower(0.8);
                runtimeWait=runtime.seconds()+0.25; //Sets a timer for .25 seconds until it can go up another setting
            }
            
            //Crane setting positions
            if(craneSetting == 0) { //COMPACT MODE
                craneElevate.setTargetPosition(0);
                cranePitch.setTargetPosition(0);
            }
            if(craneSetting == 1) { //LOWEST
                craneElevate.setTargetPosition(0);
                cranePitch.setTargetPosition(0);
            }
            if(craneSetting == 2) {
                craneElevate.setTargetPosition(0);
                cranePitch.setTargetPosition(50);
            }
            if(craneSetting == 3) {
                craneElevate.setTargetPosition(0);
                cranePitch.setTargetPosition(140);
            }
            if(craneSetting == 4) {
                craneElevate.setTargetPosition(0);
                cranePitch.setTargetPosition(280);
            }
            if(craneSetting == 5) {
                craneElevate.setTargetPosition(0);
                cranePitch.setTargetPosition(420);
            }
            if(craneSetting == 6) {
                craneElevate.setTargetPosition(180);
                cranePitch.setTargetPosition(280);
            }
            if(craneSetting == 7) {
                craneElevate.setTargetPosition(180);
                cranePitch.setTargetPosition(0);
            }
            if(craneSetting == 8) {
                craneElevate.setTargetPosition(0);
                cranePitch.setTargetPosition(0);
            }
            if(craneSetting == 10) {
                craneElevate.setTargetPosition(0);
                cranePitch.setTargetPosition(0);
            }
            if(craneSetting == 11) {
                craneElevate.setTargetPosition(0);
                cranePitch.setTargetPosition(0);
            }
            if(craneSetting == 12) { //HIGHEST
                craneElevate.setTargetPosition(0);
                cranePitch.setTargetPosition(0);
            }
            
//******************************************************************************
            
            //Servo control
            //Crane grabber
            if(triggerPowerLeft2==1) { 
                craneGrab.setPosition(0.5);
            }
            else if(gamepad2.left_bumper==true) {
                craneGrab.setPosition(0);
            }
            
            //Crane grabber attitude 
            if(gamepad2.dpad_left==true) {
                craneGrabAttitude.setPosition(0);
            }
            if(gamepad2.dpad_right==true) {
                craneGrabAttitude.setPosition(0.5);
            }
            
            //Tray grabber
            if(triggerPowerRight2==1) {
                trayGrabPos = 0.5;
                trayGrab.setPosition(0.5);
            }
            else if(gamepad2.right_bumper==true) {
                trayGrabPos = 0;
                trayGrab.setPosition(0);
            }

//******************************************************************************

            //Telemetry display
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("CrabMotors", "left (%.2f), right (%.2f)", moveLeftPower, moveRightPower);
            telemetry.addData("Crane Motors", "craneExtend (%.2f), cranePitch (%.2f)", craneExtendPower, cranePitchPower);
            telemetry.addData("Grabbers", "craneGrab (%.2f), trayGrab (%.2f)", craneGrabPos, trayGrabPos);
            telemetry.addData("Waiter thing", "" + runtimeWait);
            telemetry.addData("Crane Setting", "" + craneSetting);
            telemetry.update();
        }
        //Runs to put the robot down to default state
        craneElevate.setTargetPosition(0);
        cranePitch.setTargetPosition(0);
    }
}
