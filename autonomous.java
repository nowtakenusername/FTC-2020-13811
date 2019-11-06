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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;}}
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;


@TeleOp(name="autonomous", group="Iterative Opmode")
@Disabled
public class autonomous extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftPower = null;
    private DcMotor rightPower = null;
    private ColorSensor sensorColor = null;
    private DistanceSensor sensorDistance = null;
    int motorPower = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        //The below lines of code initialize the hardware variables to be used later on (such as motors and servos)
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");


        //this sets each motor to be run using encoders
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD); //change to reverse if buggy movement occurs
        rightDrive.setDirection(DcMotor.Direction.REVERSE); //change to forward if buggy movement occurs
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    //CRAB LEFT AND CRAB RIGHT
    public void crab(int pulses, String direction) {
        int wheelPosition = pulses;
        if (direction == "right") {
            leftDrive.setTargetPosition(wheelPosition);
            rightDrive.setTargetPosition(wheelPosition);
            leftFrontDrive.setTargetPosition(-wheelPosition);
            rightFrontDrive.setTargetPosition(-wheelPosition);
        } else if (direction == "left") {
            leftDrive.setTargetPosition(-wheelPosition);
            rightDrive.setTargetPosition(-wheelPosition);
            leftFrontDrive.setTargetPosition(wheelPosition);
            rightFrontDrive.setTargetPosition(wheelPosition);
        }
    }

    public void move(int pulses, String direction) {
        int wheelPosition = pulses;
        if (direction == "forward") {
            leftDrive.setTargetPosition(wheelPosition);
            rightDrive.setTargetPosition(wheelPosition);
            leftFrontDrive.setTargetPosition(wheelPosition);
            rightFrontDrive.setTargetPosition(wheelPosition);
        } else if (direction == "backwards") {
            leftDrive.setTargetPosition(-wheelPosition);
            rightDrive.setTargetPosition(-wheelPosition);
            leftFrontDrive.setTargetPosition(-wheelPosition);
            rightFrontDrive.setTargetPosition(-wheelPosition);
        }
    }
    void increasePower(int power) {power += power*0.1;
    }

    void setPower(int power) {
        increasePower(power);
        leftDrive.setPower(power);
        rightDrive.setPower(power);
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
    }

    void setPosition(int pulses) {
        leftDrive.setTargetPosition(pulses);
        rightDrive.setTargetPosition(pulses);
        leftFrontDrive.setTargetPosition(pulses);
        rightFrontDrive.setTargetPosition(pulses);
    }
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        crab(2240, "left");
        move(2240, "forward");

        motorPower = 0;

        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);



        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */ Log" order="4" sideWeight="0.50105375" side_tool="true" visible="true" weight="0.3293
