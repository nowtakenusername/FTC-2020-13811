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


@Autonomous(name="autonomousFWRDRIGHT", group="Iterative Opmode")
//@Disabled
public class autonomousFWRDRIGHT extends LinearOpMode
{
    //Declaring OpMode members.
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
    private DcMotor fakeMotor;
    
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

        //this sets each motor to be run using encoders
        craneExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cranePitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        craneExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        cranePitch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        //Sets motor direction
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);

        
        //Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        //Moves go here:
        cranePitch.setPower(0.8);
        cranePitch.setTargetPosition(100);
        move(5000, 0.5, "forwards", 2);
        crab(10000, 0.5, "right", 2.5);
    }
    
    
    public void move(int pulses, double power, String direction, double timeout) { //moving forwards/backwards [pulse] pulses at [power] power
        double moveStart = runtime.seconds();
        if(direction == "forwards") { //move forwards {
            leftBackDrive.setPower(-power);
            rightBackDrive.setPower(-power);
            leftFrontDrive.setPower(-power);
            rightFrontDrive.setPower(-power);
            while(-pulses<craneExtend.getCurrentPosition() && timeout+moveStart > runtime.seconds()) {
                telemetry.addData("Encoder 1", craneExtend.getCurrentPosition());
                telemetry.addData("Encoder 2", fakeMotor.getCurrentPosition());
                telemetry.update();
            }
        }
        else if(direction == "backwards") { //move backwards
            leftBackDrive.setPower(power);
            rightBackDrive.setPower(power);
            leftFrontDrive.setPower(power);
            rightFrontDrive.setPower(power);
            while(pulses>craneExtend.getCurrentPosition() && timeout+moveStart > runtime.seconds()) {
                telemetry.addData("Encoder 1", craneExtend.getCurrentPosition());
                telemetry.addData("Encoder 2", fakeMotor.getCurrentPosition());
                telemetry.update();
            }
        }
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        encoderReset();
    }

    public void crab(int pulses, double power, String direction, double timeout) { //crabbing left/right for [pulse] pulses at [power] power
        double moveStart = runtime.seconds();
        if (direction == "right") { //crab right... i think. havent tested it out
            leftBackDrive.setPower(power);
            rightBackDrive.setPower(-power);
            leftFrontDrive.setPower(-power);
            rightFrontDrive.setPower(power);
            while (-pulses < fakeMotor.getCurrentPosition() && timeout+moveStart > runtime.seconds()) {
                telemetry.addData("Encoder 1", craneExtend.getCurrentPosition());
                telemetry.addData("Encoder 2", fakeMotor.getCurrentPosition());
                telemetry.update();
            }
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            encoderReset();
        }
        else if (direction == "left") { //crab left
            leftBackDrive.setPower(-power);
            rightBackDrive.setPower(power);
            leftFrontDrive.setPower(power);
            rightFrontDrive.setPower(-power);
            while(pulses > fakeMotor.getCurrentPosition() && timeout+moveStart > runtime.seconds()) {
                telemetry.addData("Encoder 1", craneExtend.getCurrentPosition());
                telemetry.addData("Encoder 2", fakeMotor.getCurrentPosition());
                telemetry.update();
            }
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            craneExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            fakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void tray(String position) { //brings the traygrabber up/down
        if (position == "up"); {
            trayGrab.setPosition(0);
            while(trayGrab.getPosition() != 0)
                trayGrab.setPosition(0);
                sleep(5000);
        }
        if (position == "down"); {
            trayGrab.setPosition(1);
            while(trayGrab.getPosition() != 1)
                trayGrab.setPosition(1);
                sleep(5000);
        }
    }
    
    public void encoderReset() { //Resets encoder values - used inbetween moves
        craneExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
