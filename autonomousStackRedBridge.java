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


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@Autonomous(name="autonomousStackBlueBridge", group="Iterative Opmode")
//@Disabled
public class autonomousStackBlueBridge extends LinearOpMode
{
    //Declaring OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftBackDrive, rightBackDrive, leftFrontDrive, rightFrontDrive;
    private DcMotor craneExtend, cranePitch, craneElevate, fakeMotor;
    private Servo craneGrab, trayGrab, craneGrabAttitude, flipperLeft, flipperRight;
    private BNO055IMU imu;
    
    //Setting up variables
    Orientation lastAngles = new Orientation();
    double globalAngle;
    double heading = 0;
    
    @Override
    public void runOpMode() {
        //The below lines of code initialize the hardware variables to be used later on (such as motors and servos)
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        craneExtend = hardwareMap.get(DcMotor.class, "craneExtend"); //port 0, hub2
        cranePitch = hardwareMap.get(DcMotor.class, "cranePitch"); //port 1, hub2
        craneElevate = hardwareMap.get(DcMotor.class, "craneElevate"); //port 2, hub2
        craneGrab = hardwareMap.get(Servo.class, "craneGrab"); //port 0, servo
        trayGrab = hardwareMap.get(Servo.class, "trayGrab"); //port 1, servo
        fakeMotor = hardwareMap.get(DcMotor.class, "fakeMotor"); //port 3, encoder
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        
        //Setting up imu parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

    
        //this sets each motor to be run using encoders
        craneExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cranePitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        craneElevate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        craneExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        cranePitch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        craneElevate.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Sets motor direction
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);

        //initialize imu
        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();
        while (!isStopRequested() && !imu.isGyroCalibrated())
        { sleep(50); idle(); }
        
        //Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        
        //Moves go here:
        trayGrab.setPosition(0);
        move("backwards", 2000, 0.7, 1);
        crab("left", 3660, 0.5, 2);
        move("backwards", 3250, 0.7, 1.35);
        tray("down");
        move("forwards", 6300, 0.4, 6.5);
        rotate("right", 30, 0.3);
        tray("up");
        move("forwards", 150, 0.2, 0.3);
        rotate("left", 30, 0.3);
        crab("right", 500, 0.3, 1);
        crab("right", 3500, 0.5, 4.5);
        move("backwards", 3500, 0.7, 2.5);
        rotate("left", 90, 0.3);
        crane(0, -300, 0, -0.5);
        craneExtend.setPower(-1);
        sleep(250);
        craneExtend.setPower(0);
        crane(150, 0, 0.5, 0.2);
        move("backwards", 2000, 0.4, 2.5);
        
        
    }
    
    //57 pulses per inch
    public void move(String direction, int pulses, double power, double timeout) { //move [direction] [pulse] pulses at [power] power
        double moveStart = runtime.seconds();
        if(direction == "forwards") { //move forwards {
            leftBackDrive.setPower(-power);
            rightBackDrive.setPower(-power);
            leftFrontDrive.setPower(-power);
            rightFrontDrive.setPower(-power);
            while(!isStopRequested() && -pulses < craneExtend.getCurrentPosition() && timeout + moveStart > runtime.seconds()) {
                if (getAngle() > heading) {
                    leftBackDrive.setPower(-power-0.1);
                    rightBackDrive.setPower(-power+0.1);
                    leftFrontDrive.setPower(-power-0.1);
                    rightFrontDrive.setPower(-power+0.1);
                }
                if (getAngle() < heading) {
                    leftBackDrive.setPower(-power+0.1);
                    rightBackDrive.setPower(-power-0.1);
                    leftFrontDrive.setPower(-power+0.1);
                    rightFrontDrive.setPower(-power-0.1);
                }
                else {
                    leftBackDrive.setPower(-power);
                    rightBackDrive.setPower(-power);
                    leftFrontDrive.setPower(-power);
                    rightFrontDrive.setPower(-power);
                }
                telemetry.addData("Encoder 1", craneExtend.getCurrentPosition());
                telemetry.addData("Encoder 2", fakeMotor.getCurrentPosition());
                telemetry.addData("Degree" , getAngle());
                telemetry.addData("Heading" , heading);
                telemetry.update();
            }
        }
        else if(direction == "backwards") { //move backwards
            leftBackDrive.setPower(power);
            rightBackDrive.setPower(power);
            leftFrontDrive.setPower(power);
            rightFrontDrive.setPower(power);
            while(!isStopRequested() && pulses > craneExtend.getCurrentPosition() && timeout + moveStart > runtime.seconds()) {
                if (getAngle() > heading) {
                    leftBackDrive.setPower(power-0.1);
                    rightBackDrive.setPower(power);
                    leftFrontDrive.setPower(power-0.1);
                    rightFrontDrive.setPower(power);
                }
                if (getAngle() < heading) {
                    leftBackDrive.setPower(power+0.1);
                    rightBackDrive.setPower(power);
                    leftFrontDrive.setPower(power+0.1);
                    rightFrontDrive.setPower(power);
                }
                else {
                    leftBackDrive.setPower(power);
                    rightBackDrive.setPower(power);
                    leftFrontDrive.setPower(power);
                    rightFrontDrive.setPower(power);
                }
                telemetry.addData("Encoder 1", craneExtend.getCurrentPosition());
                telemetry.addData("Encoder 2", fakeMotor.getCurrentPosition());
                telemetry.addData("Degree" , getAngle());
                telemetry.addData("Heading" , heading);
                telemetry.update();
            }
        }
        resetMove();
    }

    public void crab(String direction, int pulses, double power, double timeout) { //crabs [direction] for [pulse] pulses at [power] power
        double moveStart = runtime.seconds();
        if (direction == "right") { //crab right
            leftBackDrive.setPower(power);
            rightBackDrive.setPower(-power);
            leftFrontDrive.setPower(-power);
            rightFrontDrive.setPower(power);
            while (!isStopRequested() && -pulses < fakeMotor.getCurrentPosition() && timeout + moveStart > runtime.seconds()) {
                telemetry.addData("Encoder 1", craneExtend.getCurrentPosition());
                telemetry.addData("Encoder 2", fakeMotor.getCurrentPosition());
                telemetry.addData("Degree" , getAngle());
                telemetry.addData("Heading" , heading);
                telemetry.update();
            }
        }
        else if (direction == "left") { //crab left
            leftBackDrive.setPower(-power);
            rightBackDrive.setPower(power);
            leftFrontDrive.setPower(power);
            rightFrontDrive.setPower(-power);
            while(!isStopRequested() && pulses > fakeMotor.getCurrentPosition() && timeout + moveStart > runtime.seconds()) {
                telemetry.addData("Encoder 1", craneExtend.getCurrentPosition());
                telemetry.addData("Encoder 2", fakeMotor.getCurrentPosition());
                telemetry.addData("Degree" , getAngle());
                telemetry.addData("Heading" , heading);
                telemetry.update();
            }
        }
        resetMove();
    }
    
    public void rotate(String direction, double degrees, double power) { //rotates [direction] [degrees] degrees at [power] power
        if (direction == "left") { //rotate left
            heading += degrees;
            leftBackDrive.setPower(power);
            rightBackDrive.setPower(-power);
            leftFrontDrive.setPower(power);
            rightFrontDrive.setPower(-power);
            while (!isStopRequested() && getAngle() < heading) {
                telemetry.addData("Encoder 1", craneExtend.getCurrentPosition());
                telemetry.addData("Encoder 2", fakeMotor.getCurrentPosition());
                telemetry.addData("Degree" , getAngle());
                telemetry.addData("Heading" , heading);
                telemetry.update();
            }
        }
        if (direction == "right") { //rotate right
            heading -= degrees;
            leftBackDrive.setPower(-power);
            rightBackDrive.setPower(power);
            leftFrontDrive.setPower(-power);
            rightFrontDrive.setPower(power);
            while (!isStopRequested() && getAngle() > heading) {
                telemetry.addData("Encoder 1", craneExtend.getCurrentPosition());
                telemetry.addData("Encoder 2", fakeMotor.getCurrentPosition());
                telemetry.addData("Degree" , getAngle());
                telemetry.addData("Heading" , heading);
                telemetry.update();
            }
        }
        resetMove();
    }

    public void tray(String position) {
        if (position == "up") { //traygrabber goes [position]
            while(!isStopRequested() && trayGrab.getPosition() != 0)
                trayGrab.setPosition(0);
            sleep(1500);
        }
        if (position == "down") {
            while(!isStopRequested() && trayGrab.getPosition() != 1)
                trayGrab.setPosition(1);
            sleep(1500);
        }
    }
    public void crane(int cranePitchPulses, int craneElevatePulses, double cranePitchPower2,  double craneElevatePower2) {
        cranePitch.setPower(cranePitchPower2);
        craneElevate.setPower(craneElevatePower2);
        cranePitch.setTargetPosition(cranePitchPulses);
        craneElevate.setTargetPosition(craneElevatePulses);
        sleep(2500);
    }
    public void resetMove() { //Resets encoder values and stops wheels - preps for next move
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        sleep(100);
        craneExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    
    private void resetAngle() { //Resets angle heading
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }
    
    private double getAngle() { //Converts imu z-axis heading into a proper format
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;
        lastAngles = angles;
        return globalAngle;
    }
}
