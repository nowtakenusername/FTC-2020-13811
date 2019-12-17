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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@Autonomous(name="autonomous", group="Iterative Opmode")
//@Disabled //you shouldn't touch this
public class autonomous extends OpMode
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
    private DcMotor leftPower;
    private DcMotor rightPower;
    private DcMotor fakeMotor;
    private double forwardsPower;
    private ColorSensor colorSensor;

    @Override
    public void init() { //Runs once when robot is initialized

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
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor"); //port 0, ic2 thing

        //this sets each motor to be run using encoders
        craneExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        craneExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //craneExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //fakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //sets up the gyroscope
        //gyro = hardwareMap.get();

        //Sets motor direction
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);

        //Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    //These functions allow easier programming of automonous modes...

    public void move(int pulses, double power, String direction) { //moving forwards/backwards [pulse] pulses at [power] power
        if(direction == "forwards") { //move forwards {
            leftBackDrive.setPower(-power);
            rightBackDrive.setPower(-power);
            leftFrontDrive.setPower(-power);
            rightFrontDrive.setPower(-power);
            while(-pulses<craneExtend.getCurrentPosition()) {
                telemetry.addData("Power", "power (%.2f)", forwardsPower);
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
        else if(direction == "backwards") { //move backwards
            leftBackDrive.setPower(power);
            rightBackDrive.setPower(power);
            leftFrontDrive.setPower(power);
            rightFrontDrive.setPower(power);
            while(pulses>craneExtend.getCurrentPosition()) {
                telemetry.addData("Power", "power (%.2f)", forwardsPower);
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

    public void crab(int pulses, double power, String direction) { //crabbing left/right for [pulse] pulses at [power] power
        if (direction == "right") { //crab right... i think. havent tested it out
            leftBackDrive.setPower(power);
            rightBackDrive.setPower(power);
            leftFrontDrive.setPower(-power);
            rightFrontDrive.setPower(-power);
            while (pulses > fakeMotor.getCurrentPosition()) {}
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
        }
        else if (direction == "left") { //crab left
            leftBackDrive.setPower(-power);
            rightBackDrive.setPower(-power);
            leftFrontDrive.setPower(power);
            rightFrontDrive.setPower(power);
            while(-pulses<fakeMotor.getCurrentPosition()) {}
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
        }
    }

    void setPower(double power) { //sets motor power (speed of the robot's movement)
        forwardsPower = power;
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
    }

    /*public void turn(int pulses, String direction) { //turns the robot [pulse] pulses
        if (direction == "right") { //turns right
                leftBackDrive.setTargetPosition(pulses);
                rightBackDrive.setTargetPosition(pulses);
                leftFrontDrive.setTargetPosition(pulses);
                rightFrontDrive.setTargetPosition(pulses);

        } else if (direction == "left") { //turns left

                leftBackDrive.setTargetPosition(-pulses);
                rightBackDrive.setTargetPosition(-pulses);
                leftFrontDrive.setTargetPosition(-pulses);
                rightFrontDrive.setTargetPosition(-pulses);
            }
        }*/


    public void tray(String position) { //brings the traygrabber up/down
        if (position == "up"); { //traygrab up, or default position
            trayGrab.setPosition(0);
        }
        if (position == "down"); { //traygrab down
            trayGrab.setPosition(0.5);
        }
    }

    void setPosition(int pulses) { //used for debugging purposes
        leftBackDrive.setTargetPosition(pulses);
        rightBackDrive.setTargetPosition(pulses);
        leftFrontDrive.setTargetPosition(pulses);
        rightFrontDrive.setTargetPosition(pulses);
    }

    //Loops when you press init
    @Override
    public void init_loop() {
    }

    //Runs once when you press start
    @Override
    public void start() {
        move(1000, 0.5, "forwards");

        craneExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        move(1000, 0.5, "backwards");

        craneExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        runtime.reset();
    }

    //Loops when you press start
    @Override
    public void loop() {
        //move(360, 0.25, "forwards");

        // Show the elapsed game time and wheel power.
        telemetry.addData("Power", "power (%.2f)", forwardsPower);
        telemetry.addData("Encoder 1", craneExtend.getCurrentPosition());
        telemetry.addData("Encoder 2", fakeMotor.getCurrentPosition());
    }

    //Runs once when you press stop
}
