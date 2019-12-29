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
import com.qualcomm.robotcore.hardware.GyroSensor;

@TeleOp(name="pulseTest", group="Linear Opmode")
//@Disabled
public class pulseTest extends LinearOpMode {

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
    private Servo craneAttitude;

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

        craneElevate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cranePitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        craneElevate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        cranePitch.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Initialize some picky variables
        double runtimeWait = 0;
        double pulseIncrement = 10;
        double cranePitchPower;
        double craneElevatePower;
        int cranePitchPos = 0;
        int craneElevatePos = 0;

        telemetry.addData("Status", "Initialized");
        telemetry.update(); //Done "initalizing"

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            if(gamepad2.dpad_up==true && runtimeWait<=runtime.seconds()) { //sets the lift up by [increment]
                craneElevatePos += pulseIncrement;
                craneElevate.setTargetPosition(craneElevatePos);
                runtimeWait=runtime.seconds()+0.5;
            }
            else if(gamepad2.dpad_down==true && runtimeWait<=runtime.seconds()) { //sets the lift down by [increment]
                craneElevate.setPower(0.2);
                craneElevatePos -= pulseIncrement;
                craneElevate.setTargetPosition(craneElevatePos);
                runtimeWait=runtime.seconds()+0.5;
            }
            
            
            if(gamepad2.dpad_right==true && runtimeWait<=runtime.seconds()) { //sets the arm up by [increment]
                cranePitchPos += pulseIncrement;
                cranePitch.setTargetPosition(cranePitchPos);
                runtimeWait=runtime.seconds()+0.5;
            }
            else if(gamepad2.dpad_left==true && runtimeWait<=runtime.seconds()) { //sets the arm down by [increment]
                cranePitch.setPower(0.2);
                cranePitchPos -= pulseIncrement;
                cranePitch.setTargetPosition(cranePitchPos);
                runtimeWait=runtime.seconds()+0.5;
            }
            
            if(gamepad2.a==true && runtimeWait<=runtime.seconds()) { //raises the increment by one
                pulseIncrement += 5;
                runtimeWait=runtime.seconds()+0.5;
            }
            else if(gamepad2.b==true && runtimeWait<=runtime.seconds()) { //lowers the increment by one
                pulseIncrement -= 5;
                runtimeWait=runtime.seconds()+0.5;
            }

            //Shows the elapsed game time and other data on the android device.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Pulses", "pitch " + cranePitchPos);
            telemetry.addData("Pulses", "elevate " + craneElevatePos);
            telemetry.addData("Increment", " " + pulseIncrement);
            telemetry.update();
        }
        craneElevate.setTargetPosition(0);
        cranePitch.setTargetPosition(0);
    }
}
