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
 * list of conditions and http://192.168.49.1:8080/java/editor.html?/src/org/firstinspires/ftc/robotcontroller/external/samples/gamepadTank.javathe following disclaimer in the documentation and/or
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

@TeleOp(name="gamepadNC", group="Linear Opmode")
//@Disabled
public class gamepadNC extends LinearOpMode {

    //Declaring OpMode members
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftFrontDrive;
    private DcMotor craneExtend; //Used for encoder wheel forwards
    private DcMotor cranePitch;
    private DcMotor craneElevate;
    private DcMotor fakeMotor; //Used for encoder wheel sideways
    private Servo craneGrab;
    private Servo trayGrab;
    private Servo craneGrabAttitude;
    private Servo flipperLeft;
    private Servo flipperRight;

    public void runOpMode() {
        
//******************************************************************************

        //Initial setup
        //Initialize the motors, servos, to their ports. Servos are all on hub1.
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive"); //port 0, hub1
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive"); //port 1, hub1
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive"); //port 2, hub1
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive"); //port 3, hub1
        craneExtend = hardwareMap.get(DcMotor.class, "craneExtend"); //port 0, hub2
        cranePitch = hardwareMap.get(DcMotor.class, "cranePitch"); //port 1, hub2
        craneElevate = hardwareMap.get(DcMotor.class, "craneElevate"); //port 2, hub2
        fakeMotor = hardwareMap.get(DcMotor.class, "fakeMotor"); //port 2, hub3
        craneGrab = hardwareMap.get(Servo.class, "craneGrab"); //port 0, servo
        craneGrabAttitude = hardwareMap.get(Servo.class, "craneGrabAttitude"); //port 2, servo
        trayGrab = hardwareMap.get(Servo.class, "trayGrab"); //port 1, servo
        flipperLeft = hardwareMap.get(Servo.class, "flipperLeft"); //port 3. servo
        flipperRight = hardwareMap.get(Servo.class, "flipperRight"); //port 4, servo
        
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

        //Initializing variables
        //Timer variables
        double settingWait = 0; 
        double craneGrabWait = 0;
        double craneGrabAttitudeWait = 0;
        //Setting variables
        double craneGrabSetting = 0;
        double CGAD = 0.5; //craneGrabAttitudeDisplacement, or CGAD
        double trayGrabWait = 0;
        double trayGrabSetting = 0;
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
            //Setting up variables - many are redundant
            double craneExtendPower; //Crane linear actuator motor power
            double cranePitchPower; //Crane pitch motor power
            double craneElevatePower; //Crane lift motor power
            double craneGrabPos = 0; //Grabber claw servo position
            double trayGrabPos = 0; //Tray grabber servo position
            double craneGrabAttitudePos = 0; //Grabber attitude servo position
            double cranePosOffset = 0; //(un)used to offset future values if starting at craneSetting 0

//******************************************************************************

            //Drive controls
            if(gamepad1.left_trigger == 1 && gamepad1.right_trigger == 1) {
                leftBackDrive.setPower(-0.5);
                rightBackDrive.setPower(0.5);
                leftFrontDrive.setPower(-0.5);
                rightFrontDrive.setPower(0.5); 
            }
            else if(gamepad1.left_trigger == 1) {
                leftBackDrive.setPower(-0.5);
                rightBackDrive.setPower(-0.5);
                leftFrontDrive.setPower(-0.5);
                rightFrontDrive.setPower(-0.5);
            }
            else if(gamepad1.right_trigger == 1) {
                leftBackDrive.setPower(0.5);
                rightBackDrive.setPower(0.5);
                leftFrontDrive.setPower(0.5);
                rightFrontDrive.setPower(0.5);
            }
            
            else if(gamepad1.right_bumper && gamepad1.left_bumper) {
                leftBackDrive.setPower(-0.5);
                rightBackDrive.setPower(0.5);
                leftFrontDrive.setPower(-0.5);
                rightFrontDrive.setPower(0.5);
            }
            else if(gamepad1.left_bumper) {
                leftBackDrive.setPower(-0.5);
                rightBackDrive.setPower(0.5);
                leftFrontDrive.setPower(0.5);
                rightFrontDrive.setPower(-0.5);
            }
            else if(gamepad1.right_bumper) {
                leftBackDrive.setPower(0.5);
                rightBackDrive.setPower(-0.5);
                leftFrontDrive.setPower(-0.5);
                rightFrontDrive.setPower(0.5);
            }
            
            else if(gamepad1.left_stick_x != 0) {
                leftBackDrive.setPower(gamepad1.left_stick_x*0.5);
                rightBackDrive.setPower(-gamepad1.left_stick_x*0.5);
                leftFrontDrive.setPower(gamepad1.left_stick_x*0.5);
                rightFrontDrive.setPower(-gamepad1.left_stick_x*0.5);
            }
            
            else if(gamepad1.right_stick_y>0.7 || gamepad1.right_stick_y<-0.7) {
                    rightBackDrive.setPower(gamepad1.right_stick_y/1.5); //Full throttle
                    rightFrontDrive.setPower(gamepad1.right_stick_y/1.5);   }
            else {
                leftBackDrive.setPower(0);
                rightBackDrive.setPower(0);
                leftFrontDrive.setPower(0);
                rightFrontDrive.setPower(0);
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
            if(gamepad2.dpad_down==true && settingWait<=runtime.seconds() && craneSetting > 0) { 
                craneSetting -= 1; 
                craneElevate.setPower(-0.2);
                cranePitch.setPower(0.6);
                settingWait=runtime.seconds()+0.25; //Sets a timer for .25 seconds until it can go down another setting
            }
            //sets the automated crane setting up by one
            else if(gamepad2.dpad_up==true && settingWait<=runtime.seconds() && craneSetting < 12) { 
                craneSetting += 1; 
                craneElevate.setPower(-0.5);
                cranePitch.setPower(0.8);
                settingWait=runtime.seconds()+0.25; //Sets a timer for .25 seconds until it can go up another setting
            }
            
            //Crane setting positions
            if(craneSetting == 0) { //LOWEST
                craneElevate.setTargetPosition(0);
                cranePitch.setTargetPosition(-100);
            }
            if(craneSetting == 1) {
                craneElevate.setTargetPosition(0);
                cranePitch.setTargetPosition(-70);
            }
            if(craneSetting == 2) {
                craneElevate.setTargetPosition(0);
                cranePitch.setTargetPosition(20);
            }
            if(craneSetting == 3) {
                craneElevate.setTargetPosition(0);
                cranePitch.setTargetPosition(120);
            }
            if(craneSetting == 4) {
                craneElevate.setTargetPosition(0);
                cranePitch.setTargetPosition(250);
            }
            if(craneSetting == 5) {
                craneElevate.setTargetPosition(0);
                cranePitch.setTargetPosition(400);
            }
            if(craneSetting == 6) {
                craneElevate.setTargetPosition(-80);
                cranePitch.setTargetPosition(400);
            }
            if(craneSetting == 7) {
                craneElevate.setTargetPosition(-160);
                cranePitch.setTargetPosition(400);
            }
            if(craneSetting == 8) {
                craneElevate.setTargetPosition(-240);
                cranePitch.setTargetPosition(400);
            }
            if(craneSetting == 10) {
                craneElevate.setTargetPosition(-320);
                cranePitch.setTargetPosition(400);
            }
            if(craneSetting == 11) {
                craneElevate.setTargetPosition(-400);
                cranePitch.setTargetPosition(400);
            }
            if(craneSetting == 12) { //HIGHEST
                craneElevate.setTargetPosition(-480);
                cranePitch.setTargetPosition(400);
            }
            
//******************************************************************************
            
            //Servo control
            //Crane grabber
            if(gamepad2.left_trigger==1 && craneGrabWait<=runtime.seconds() && craneGrabSetting == 0) { 
                craneGrab.setPosition(0.5);
                craneGrabSetting = 1;
                craneGrabWait=runtime.seconds()+0.25;
            }
            else if(gamepad2.left_trigger==1 && craneGrabWait<=runtime.seconds() && craneGrabSetting == 1) {
                craneGrab.setPosition(0);
                craneGrabSetting = 0;
                craneGrabWait=runtime.seconds()+0.25;
            }
            
            //Crane grabber attitude 
            if(gamepad2.left_bumper==true && craneGrabAttitudeWait <= runtime.seconds() && CGAD >= 0) {
                CGAD -= 0.05;
                craneGrabAttitudeWait=runtime.seconds()+0.1;
                craneGrabAttitude.setPosition(CGAD);
            }
            else if(gamepad2.right_bumper==true && craneGrabAttitudeWait <= runtime.seconds() &&CGAD <= 1) {
                CGAD += 0.05;
                craneGrabAttitudeWait=runtime.seconds()+0.1;
                craneGrabAttitude.setPosition(CGAD);
            }
            else if(gamepad2.left_bumper==true && gamepad2.right_bumper == true) {
                CGAD = 1;
                craneGrabAttitude.setPosition(CGAD);
            }
            
            //Tray grabber
            if(gamepad2.right_trigger==1 && trayGrabWait<=runtime.seconds() && trayGrabSetting == 0) { 
                trayGrab.setPosition(1);
                trayGrabSetting = 1;
                trayGrabWait=runtime.seconds()+0.25;
            }
            else if(gamepad2.right_trigger==1 && trayGrabWait<=runtime.seconds() && trayGrabSetting == 1) {
                trayGrab.setPosition(0);
                trayGrabSetting = 0;
                trayGrabWait=runtime.seconds()+0.25;
            }
            
            //Front flipper left
            if(gamepad1.left_stick_y == -1 && (gamepad1.left_bumper || gamepad1.right_bumper)) {
                flipperLeft.setPosition(0.5); //deployed
            }
            else if(gamepad1.left_stick_x == -1 && (gamepad1.left_bumper || gamepad1.right_bumper)) {
                flipperLeft.setPosition(0); //out
            }
            else if(gamepad1.left_stick_x == 1 && (gamepad1.left_bumper || gamepad1.right_bumper)) {
                flipperLeft.setPosition(1); //in
            }
            //Front flipper right
            if(gamepad1.right_stick_y == -1 && (gamepad1.left_bumper || gamepad1.right_bumper)) {
                flipperRight.setPosition(0.4);
            }
            else if(gamepad1.right_stick_x == 1 && (gamepad1.left_bumper || gamepad1.right_bumper)) {
                flipperRight.setPosition(1);
            }
            else if(gamepad1.right_stick_x == -1 && (gamepad1.left_bumper || gamepad1.right_bumper)) {
                flipperRight.setPosition(0);
            }

//******************************************************************************

            //Telemetry display
            telemetry.addData("Run Time:", "" + runtime.toString());
            telemetry.addData("Motor Power", "L (%.2f), R (%.2f)", gamepad1.left_stick_y, gamepad1.right_stick_y);
            telemetry.addData("Crane Setting", "" + craneSetting);
            telemetry.update();
        }
    }
}
