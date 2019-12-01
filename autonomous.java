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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@Autonomous(name="autonomousLeft", group="Iterative Opmode")
//@Disabled //you shouldn't touch this
public class autonomous extends OpMode
{
    //Declaring OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftPower;
    private DcMotor rightPower;
    private double forwardsPower;
    private Servo trayGrab;

    @Override
    public void init() { //Runs once when robot is initialized

        //The below lines of code initialize the hardware variables to be used later on (such as motors and servos)
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        trayGrab = hardwareMap.get(Servo.class, "trayGrab");

        //this sets each motor to be run using encoders
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //sets up the gyroscope
        //gyro = hardwareMap.get();

        //Sets motor direction
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    //These functions allow easier programming of automonous modes...

    public void move(int pulses, String direction) { //moving forwards/backwards [pulse] pulses
        int wheelPosition = pulses;
        if (direction == "forward") { //go forward
            while(pulses>leftBackDrive.getCurrentPosition()) { //Keep in mind that leftbackdrive is a test variable until we get encoder wheels
                leftBackDrive.setTargetPosition(wheelPosition);
                rightBackDrive.setTargetPosition(-wheelPosition);
                leftFrontDrive.setTargetPosition(wheelPosition);
                rightFrontDrive.setTargetPosition(-wheelPosition);
            }
        }
        else if (direction == "backwards") { //go backwards
            while(pulses>leftBackDrive.getCurrentPosition()) {
                leftBackDrive.setTargetPosition(-wheelPosition);
                rightBackDrive.setTargetPosition(wheelPosition);
                leftFrontDrive.setTargetPosition(-wheelPosition);
                rightFrontDrive.setTargetPosition(wheelPosition);
            }
        }
    }

    public void crab(int pulses, String direction) { //crabbing left/right [pulse] pulses
        int wheelPosition = pulses;
        if (direction == "right") { //crab right
            while(pulses>leftBackDrive.getCurrentPosition()) {
                leftBackDrive.setTargetPosition(wheelPosition);
                rightBackDrive.setTargetPosition(wheelPosition);
                leftFrontDrive.setTargetPosition(-wheelPosition);
                rightFrontDrive.setTargetPosition(-wheelPosition);
            }
        } else if (direction == "left") { //crab left
            while(pulses>leftBackDrive.getCurrentPosition()) {
                leftBackDrive.setTargetPosition(-wheelPosition);
                rightBackDrive.setTargetPosition(-wheelPosition);
                leftFrontDrive.setTargetPosition(wheelPosition);
                rightFrontDrive.setTargetPosition(wheelPosition);
            }
        }
    }

    void setPower(double power) { //sets motor power (speed of the robot's movement)
        forwardsPower = power;
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
    }

    public void turn(int pulses, String direction) { //turns the robot [pulse] pulses
        if (direction == "right") { //turns right
            while(pulses>leftBackDrive.getCurrentPosition()) {
                leftBackDrive.setTargetPosition(pulses);
                rightBackDrive.setTargetPosition(pulses);
                leftFrontDrive.setTargetPosition(pulses);
                rightFrontDrive.setTargetPosition(pulses);
            }
        } else if (direction == "left") { //turns left
            while(pulses>leftBackDrive.getCurrentPosition()) {
                leftBackDrive.setTargetPosition(-pulses);
                rightBackDrive.setTargetPosition(-pulses);
                leftFrontDrive.setTargetPosition(-pulses);
                rightFrontDrive.setTargetPosition(-pulses);
            }
        }
    }

    public void tray(String position) { //brings the traygrabber up/down
        if (position == "up"); { //traygrab up, or default position
            trayGrab.setPosition(0);
        }
        else if (position == "down"); { //traygrab down
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
        setPower(0.5);
        move(500, "forward");
        setPower(0.2);
        turn("right");
        setPower(0);
        runtime.reset();
    }

    //Loops when you press start
    @Override
    public void loop() {

        // Show the elapsed game time and wheel power.
        telemetry.addData("Power", "power (%.2f)", forwardsPower);
    }

    //Runs once when you press stop
}
