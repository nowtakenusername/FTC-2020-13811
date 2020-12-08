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

//top
// Directory
package org.firstinspires.ftc.teamcode;

// Imports
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

@Autonomous(name="autonomous", group="Iterative Opmode")
// @Disabled
public class autonomous extends LinearOpMode
{
    // Declaring OpMode objects...
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftBackDrive, rightBackDrive, leftFrontDrive, rightFrontDrive;
    private DcMotor craneExtend, cranePitch, craneElevate, fakeMotor;
    private Servo ringFeeler, goalClamp, goalDeploy, flipperLeft, flipperRight;
    private BNO055IMU imu;
    
    // Setting up variables...
    Orientation lastAngles = new Orientation();
    double globalAngle;
    double heading = 0;
    public double feelState = 1;
    @Override
    public void runOpMode() { // 13811 Aledo, Texas
        // Assign the hardwareMap to be used later on
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        craneExtend = hardwareMap.get(DcMotor.class, "craneExtend"); // control hub port 0, motor encoder
        fakeMotor = hardwareMap.get(DcMotor.class, "fakeMotor"); // control hub port 1, motor encoder
        ringFeeler = hardwareMap.get(Servo.class, "ringFeeler"); // control hub port 0
        goalClamp = hardwareMap.get(Servo.class, "goalClamp"); // control hub 
        goalDeploy = hardwareMap.get(Servo.class, "goalDeploy");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        
        // Setting up imu parameters and config...
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
    
        // Set up odometry wheels
        craneExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        craneExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Set motor directions
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        
        // Initialize the imu...
        imu.initialize(parameters);
        // and calibrate the gyro...
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();
        while (!isStopRequested() && !imu.isGyroCalibrated())
        { sleep(50); idle(); }
        // and done.
        
        // Telemetry displays that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        //You a wizard Harri(son)
        // Moves go here:
        // and REMEMBER TO FOLLOW SYNTAX!!
        move("forward", 47*40, 0.5, 100);
        feel();
        
        if(feelState == 0) { //no donuts :(
            move("forward", 38 * 47,0.5, 100);
            move("right", 22 * 47,0.5, 100);
            drop();
            move("left", 22 * 47,0.5, 100);
            move("backward", 15 * 47,0.5, 100);
        }
        else if(feelState == 4) { //four donut
            move("forward", 83 * 47,0.5, 100);
            move("right", 22 * 47,0.5, 100);
            drop();
            move("left", 22 * 47,0.5, 100);
            move("backward", 60 * 47,0.5, 100);
        }
        else { //one donut
          move("forward", 63 * 47,0.5, 100);
            drop();
            move("backward", 40 * 47,0.5, 100);
        }
        
        
        move("forward", 23 * 47, 0.5, 100);
        //remove the line above once the if statements work
        move("right", 12 * 47,0.5, 100);
        shoot("high");
        shoot("high");
        shoot("high");
        move("forward", 47*10,0.5,100);
    }
    
    public void drop(){ //drop off the wobble goal
        goalDeploy.setPosition(1);
        sleep(1000); // 1 seconds
        goalClamp.setPosition(1);
        sleep(50);
        goalDeploy.setPosition(0);
        sleep(1000);
        goalClamp.setPosition(0);
    }
    public void shoot(String goal){ //shoot the rings at a goal
        if(goal == "low") { //shoot for the low goal
            
        }
        else if(goal == "mid") { //shoot for mid goal
            
        }
        else if(goal == "high"){//shoot for high goal
            
        }
        else { //shoot for power shot targets
          
        }
    }
    public void feel(){ // feel amount of rings and change feelState
        ringFeeler.setPosition(0.9); //we're going down
        if(ringFeeler.getPosition() > 0.5 && ringFeeler.getPosition() < 0.7) { //four rings
            feelState = 4;
        }
        else if(ringFeeler.getPosition() >= 0.7 && ringFeeler.getPosition() < 0.8) { //one ring
            feelState = 1;
        }
        else { //zero rings
            feelState = 0;
        }
        ringFeeler.setPosition(0); //back up please
    }
    public void move(String direction, int pulses, double power, double timeout) { //NEED WORK ON MOVING DIAGONALLY OR JUST DO CRAB & MOVE
        // Move [direction] [pulse] pulses at [power] power.
        double moveStart = runtime.seconds();
        if(direction == "forward") { // Move forward 
            leftBackDrive.setPower(-power); // Yes, forward is backward...
            rightBackDrive.setPower(-power);
            leftFrontDrive.setPower(-power);
            rightFrontDrive.setPower(-power);
            while(!isStopRequested() && pulses > craneExtend.getCurrentPosition() && timeout + moveStart > runtime.seconds()) {
                // If we are NOT stopped, and desired pulses are more than what our wheels register, we have NOT exceeded our timeout, then continue.
                if (getAngle() > heading) { // Very basic correction segment.
                    leftBackDrive.setPower(-power-0.05);
                    rightBackDrive.setPower(-power+0.05);
                    leftFrontDrive.setPower(-power-0.05);
                    rightFrontDrive.setPower(-power+0.05);
                }
                if (getAngle() < heading) {
                    leftBackDrive.setPower(-power+0.05);
                    rightBackDrive.setPower(-power-0.05);
                    leftFrontDrive.setPower(-power+0.05);
                    rightFrontDrive.setPower(-power-0.05);
                }
                else {
                    leftBackDrive.setPower(-power);
                    rightBackDrive.setPower(-power);
                    leftFrontDrive.setPower(-power);
                    rightFrontDrive.setPower(-power);
                }
                // Forcibly update telemetry in the move.
                telemetry.addData("Encoder 1", craneExtend.getCurrentPosition());
                telemetry.addData("Encoder 2", fakeMotor.getCurrentPosition());
                telemetry.addData("Degree" , getAngle());
                telemetry.addData("Heading" , heading);
                telemetry.update();
            }
        }
        else if(direction == "backward") { // Move backward
            leftBackDrive.setPower(power); // Once again, backward is positive power and vice versa...
            rightBackDrive.setPower(power);
            leftFrontDrive.setPower(power);
            rightFrontDrive.setPower(power);
            while(!isStopRequested() && -pulses < craneExtend.getCurrentPosition() && timeout + moveStart > runtime.seconds()) {
                // If we are NOT stopped, and negative desired pulses are LESS than what our wheels register, we have NOT exceeded our timeout, then continue.
                if (getAngle() > heading) { // Very basic correction segment.
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
                // Forcibly update telemetry in the move.
                telemetry.addData("Encoder 1", craneExtend.getCurrentPosition());
                telemetry.addData("Encoder 2", fakeMotor.getCurrentPosition());
                telemetry.addData("Degree" , getAngle());
                telemetry.addData("Heading" , heading);
                telemetry.update();
            }
        }
        else if (direction == "right") { // Move right
            leftBackDrive.setPower(power); //Set the power like this...
            rightBackDrive.setPower(-power);
            leftFrontDrive.setPower(-power);
            rightFrontDrive.setPower(power);
            while (!isStopRequested() && -pulses < fakeMotor.getCurrentPosition() && timeout + moveStart > runtime.seconds()) {
                // If we are NOT stopped, and negative desired pulses are LESS than what our wheels register, we have NOT exceeded our timeout, then continue.
                telemetry.addData("Encoder 1", craneExtend.getCurrentPosition()); // FORCIBLY update telemetry in the move.
                telemetry.addData("Encoder 2", fakeMotor.getCurrentPosition());
                telemetry.addData("Degree" , getAngle());
                telemetry.addData("Heading" , heading);
                telemetry.update(); 
            }
        }
        else if (direction == "left") { //Move left
            leftBackDrive.setPower(-power);
            rightBackDrive.setPower(power);
            leftFrontDrive.setPower(power);
            rightFrontDrive.setPower(-power);
            while(!isStopRequested() && pulses > fakeMotor.getCurrentPosition() && timeout + moveStart > runtime.seconds()) {
                telemetry.addData("Encoder 1", craneExtend.getCurrentPosition()); // FORCIBLY update telemetry in the move.
                telemetry.addData("Encoder 2", fakeMotor.getCurrentPosition());
                telemetry.addData("Degree" , getAngle());
                telemetry.addData("Heading" , heading);
                telemetry.update();
            }
        }
        resetMove(); // Reset our odometry and stop the wheels.
    }
    /*
    public void strafe(String direction, int pulses, double power, double timeout) { 
        // Strafe [direction] for [pulse] pulses at [power] power
        double moveStart = runtime.seconds();
        if (direction == "right") { // Strafe right
            leftBackDrive.setPower(power); //Set the power like this...
            rightBackDrive.setPower(-power);
            leftFrontDrive.setPower(-power);
            rightFrontDrive.setPower(power);
            while (!isStopRequested() && -pulses < fakeMotor.getCurrentPosition() && timeout + moveStart > runtime.seconds()) {
                // If we are NOT stopped, and negative desired pulses are LESS than what our wheels register, we have NOT exceeded our timeout, then continue.
                telemetry.addData("Encoder 1", craneExtend.getCurrentPosition()); // FORCIBLY update telemetry in the move.
                telemetry.addData("Encoder 2", fakeMotor.getCurrentPosition());
                telemetry.addData("Degree" , getAngle());
                telemetry.addData("Heading" , heading);
                telemetry.update(); 
            }
        }
        else if (direction == "left") { //move the robot like a crab to the left
            leftBackDrive.setPower(-power);
            rightBackDrive.setPower(power);
            leftFrontDrive.setPower(power);
            rightFrontDrive.setPower(-power);
            while(!isStopRequested() && pulses > fakeMotor.getCurrentPosition() && timeout + moveStart > runtime.seconds()) {
                telemetry.addData("Encoder 1", craneExtend.getCurrentPosition()); // FORCIBLY update telemetry in the move.
                telemetry.addData("Encoder 2", fakeMotor.getCurrentPosition());
                telemetry.addData("Degree" , getAngle());
                telemetry.addData("Heading" , heading);
                telemetry.update();
            }
        }
        resetMove(); // Reset our odometry and stop the wheels.
    }
    */
    // This will allow the robot to move DIAGONALLY in autonomous, rough stuff to do but it can be done
    // public void move(double driveX, double driveY, int pulses, double power, double timeout) { // Set a [vector] and move [pulse] pulses at [power] power
    //     double moveStart = runtime.seconds(); // Used for the timeout variable.
    //     // Set the vector!
    //     //BUT LATER
        
    //     double speed = 0.5;
    //     leftBackDrive.setPower((driveX - driveZ) - driveTurn);
    //     rightBackDrive.setPower((driveX + driveZ) + driveTurn);
    //     leftFrontDrive.setPower((driveX + driveZ) - driveTurn);
    //     rightFrontDrive.setPower((driveX - driveZ) + driveTurn);
    //     }
    //     resetMove();
    // }
    
    public void rotate(String direction, double degrees, double power) { 
        // Rotate [direction] [degrees] degrees at [power] power
        if (direction == "left") { // Rotate left
            heading += degrees; //(h = h + d) Setting a heading for correction purposes
            leftBackDrive.setPower(power);
            rightBackDrive.setPower(-power);
            leftFrontDrive.setPower(power);
            rightFrontDrive.setPower(-power);
            while (!isStopRequested() && getAngle() < heading) { // FORCIBLY update telemetry
                telemetry.addData("Encoder 1", craneExtend.getCurrentPosition());
                telemetry.addData("Encoder 2", fakeMotor.getCurrentPosition());
                telemetry.addData("Degree" , getAngle());
                telemetry.addData("Heading" , heading);
                telemetry.update();
            }
        }
        if (direction == "right") { // Rotate right
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
        resetMove();// Reset our odometry and stop the wheels.
    }

    public void resetMove() { // Reset encoder values and stop wheels
        leftBackDrive.setPower(0); // Zero them out
        rightBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        sleep(100); // Pause a bit
        craneExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the encoders.
        fakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void resetAngle() { // Resets angle heading
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }
    
    private double getAngle() { // Converts imu z-axis heading into a proper format
    // NOTE: I did copy this from the FTC website. No clue how it works, but it does!
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
} // The END
