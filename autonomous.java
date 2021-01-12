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
    private DcMotor launcherLeft, launcherRight, launcherElevate, conveyorDrive;
    private Servo ringFeeler, goalClamp, goalDeploy, conveyorLeft, conveyorRight;
    private BNO055IMU imu;
    
    // Setting up variables...
    Orientation lastAngles = new Orientation();
    double globalAngle;
    double heading = 0;
    public double feelState = 1;
    @Override
    public void runOpMode() {
        // Assign the hardwareMap to be used later on.
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive"); // control hub port 0
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive"); // control hub port 1
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive"); // control hub port 2
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive"); // control hub port 2
        launcherLeft = hardwareMap.get(DcMotor.class, "launcherLeft"); // expansion hub port 0
        launcherRight = hardwareMap.get(DcMotor.class, "launcherRight"); // expansion hub port 1
        conveyorDrive = hardwareMap.get(DcMotor.class, "conveyorDrive"); // expansion hub port 2
        ringFeeler = hardwareMap.get(Servo.class, "ringFeeler"); // control hub servo port 0
        goalClamp = hardwareMap.get(Servo.class, "goalLeft"); // control hub servo port 1
        goalDeploy = hardwareMap.get(Servo.class, "goalRight"); // control hub servo port 2
        conveyorLeft = hardwareMap.get(Servo.class, "conveyorLeft"); // expansion hub servo port 0
        conveyorRight = hardwareMap.get(Servo.class, "conveyorRight"); // expansion hub servo port 1
        imu = hardwareMap.get(BNO055IMU.class, "imu"); // core internal module in control hub
        
        // Setting up imu parameters...
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false; // Don't fill the log up with garbage...
    
        // Settng up odometry wheels...
        launcherLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // We use the launcher rollers as our encoder port donors...
        launcherRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // It's the simplest way we can do it.
        launcherLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Set motor directions...
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD); // The motor directions are seriously messed up.
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE); // Forwards is backwards, and vice versa.
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD); // But, since it works pretty nicely with the invertedness of the gamepad's sticks,
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE); // we keep it like this.
        launcherLeft.setDirection(DcMotor.Direction.FORWARD);
        launcherRight.setDirection(DcMotor.Direction.REVERSE);
        
        // Initialize the imu
        imu.initialize(parameters);
        // and calibrate the gyro...
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();
        while (!isStopRequested() && !imu.isGyroCalibrated()) // Just ensure that it REALLY calibrates...
        { sleep(50); idle(); }
        // and done.
        
        // Telemetry displays that the initialization is complete
        telemetry.addData("Status", "Initialized");
        telemetry.update(); // and update, of course.

        waitForStart(); // Wait for the game to start.
        runtime.reset(); // Reset runtime. We don't have to do this, but it's useful for logging purposes.
        
        // Moves go here:
        // As a bit of background, since this code is different than many other teams, we use functions with (dangerous)
        // while loops to ensure that the move is completed before we move onto the next one. The functions have a few
        // parameters to allow for simple programming and rapid turnaround for our autonomous. This is the same method
        // we used last year, and we see no reason in changing it for the meantime until we get Android Studio working.
        // Moves will be explained in their respective function bodies.
        move("forward", 47*40, 0.5, 100);
        feel();
        if(feelState == 0) { // No donuts :(
            move("forward", 38 * 47,0.5, 100);
            move("right", 22 * 47,0.5, 100);
            drop();
            move("left", 22 * 47,0.5, 100);
            move("backward", 15 * 47,0.5, 100);
        }
        else if(feelState == 4) { // Four donuts
            move("forward", 83 * 47,0.5, 100);
            move("right", 22 * 47,0.5, 100);
            drop();
            move("left", 22 * 47,0.5, 100);
            move("backward", 60 * 47,0.5, 100);
        }
        else { // One donut
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
    
    public void move(String direction, int pulses, double power, double timeout) {
        // Move [direction] [pulse] pulses at [power] power.
        double moveStart = runtime.seconds(); // Prepare for the timeout feature in the logic to come later.
        if(direction == "forward") { // If we are moving forward...
            leftBackDrive.setPower(-power); // Yes, forward is indeed backward.
            rightBackDrive.setPower(-power);
            leftFrontDrive.setPower(-power);
            rightFrontDrive.setPower(-power);
            while(!isStopRequested() && pulses > launcherLeft.getCurrentPosition() && timeout + moveStart > runtime.seconds()) {
                // If we are NOT stopped, and desired pulses are more than what our wheels register, and we have not registered a timeout, then continue.
                if (getAngle() > heading) { // This statement attempts to correct the robot to its heading if it is not already corrected.
                    leftBackDrive.setPower(-power-0.05); // Keep in mind that this will create a sort of wiggling motion as the robot is in motion.
                    rightBackDrive.setPower(-power+0.05); // This is unavoidable unless we use a PID, which is a serious challenge to set up.
                    leftFrontDrive.setPower(-power-0.05);
                    rightFrontDrive.setPower(-power+0.05);
                }
                if (getAngle() < heading) {
                    leftBackDrive.setPower(-power+0.05);
                    rightBackDrive.setPower(-power-0.05);
                    leftFrontDrive.setPower(-power+0.05);
                    rightFrontDrive.setPower(-power-0.05);
                }
                else { // This else statement, believe it or not, will never execute.
                    leftBackDrive.setPower(-power);
                    rightBackDrive.setPower(-power);
                    leftFrontDrive.setPower(-power);
                    rightFrontDrive.setPower(-power);
                }
                // Forcibly update telemetry in the move.
                telemetry.addData("Encoder 1", launcherLeft.getCurrentPosition());
                telemetry.addData("Encoder 2", launcherRight.getCurrentPosition());
                telemetry.addData("Degree" , getAngle());
                telemetry.addData("Heading" , heading);
                telemetry.update();
            }
        }
        else if(direction == "backward") { // If we are moving backward...
            leftBackDrive.setPower(power); // Once again, backward is positive power and vice versa...
            rightBackDrive.setPower(power); // and yes, I am too afraid to change it.
            leftFrontDrive.setPower(power);
            rightFrontDrive.setPower(power);
            while(!isStopRequested() && -pulses < launcherLeft.getCurrentPosition() && timeout + moveStart > runtime.seconds()) {
                // If we are NOT stopped, and negative desired pulses are less than what our wheels register, and we have not registered a timeout, then continue.
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
                // Forcibly update telemetry in the move.
                telemetry.addData("Encoder 1", launcherLeft.getCurrentPosition());
                telemetry.addData("Encoder 2", launcherRight.getCurrentPosition());
                telemetry.addData("Degree" , getAngle());
                telemetry.addData("Heading" , heading);
                telemetry.update();
            }
        }
        else if (direction == "right") { // If we are moving right...
            leftBackDrive.setPower(power); // Set the power like this...
            rightBackDrive.setPower(-power);
            leftFrontDrive.setPower(-power);
            rightFrontDrive.setPower(power);
            while (!isStopRequested() && -pulses < launcherRight.getCurrentPosition() && timeout + moveStart > runtime.seconds()) {
            // Same situation as the backward logic, but sideways. 
            // Making the correction function for this would make my brain explode, unfortunately.
                telemetry.addData("Encoder 1", launcherLeft.getCurrentPosition()); // FORCIBLY update telemetry in the move.
                telemetry.addData("Encoder 2", launcherRight.getCurrentPosition());
                telemetry.addData("Degree" , getAngle());
                telemetry.addData("Heading" , heading);
                telemetry.update(); 
            }
        }
        else if (direction == "left") { // If we are moving left...
            leftBackDrive.setPower(-power); // Set the power like this...
            rightBackDrive.setPower(power);
            leftFrontDrive.setPower(power);
            rightFrontDrive.setPower(-power);
            while(!isStopRequested() && pulses > launcherRight.getCurrentPosition() && timeout + moveStart > runtime.seconds()) {
            // Same situation as the forward logic, but sideways. 
                telemetry.addData("Encoder 1", launcherLeft.getCurrentPosition()); // FORCIBLY update telemetry in the move.
                telemetry.addData("Encoder 2", launcherRight.getCurrentPosition());
                telemetry.addData("Degree" , getAngle());
                telemetry.addData("Heading" , heading);
                telemetry.update();
            }
        }
        resetMove();
    }
    
    public void rotate(String direction, double degrees, double power) { 
        // Rotate [direction] [degrees] degrees at [power] power
        // There is no timeout, as if we somehow fail to reach our targeted angle the autonomous is ruined anyway.
        if (direction == "left") { // If we are rotating left...
            heading += degrees; // Setting a heading for correction purposes
            leftBackDrive.setPower(power);
            rightBackDrive.setPower(-power);
            leftFrontDrive.setPower(power);
            rightFrontDrive.setPower(-power);
            while (!isStopRequested() && getAngle() < heading) { // FORCIBLY update telemetry, but also make sure we don't go on yet.
                telemetry.addData("Encoder 1", launcherLeft.getCurrentPosition());
                telemetry.addData("Encoder 2", launcherRight.getCurrentPosition());
                telemetry.addData("Degree" , getAngle());
                telemetry.addData("Heading" , heading);
                telemetry.update();
            }
        }
        if (direction == "right") { // If we are rotating right...
            heading -= degrees; // Setting a heading for correction purposes
            leftBackDrive.setPower(-power);
            rightBackDrive.setPower(power);
            leftFrontDrive.setPower(-power);
            rightFrontDrive.setPower(power);
            while (!isStopRequested() && getAngle() > heading) { // FORCIBLY update telemetry, but also make sure we don't go on yet.
                telemetry.addData("Encoder 1", launcherLeft.getCurrentPosition());
                telemetry.addData("Encoder 2", launcherRight.getCurrentPosition());
                telemetry.addData("Degree" , getAngle());
                telemetry.addData("Heading" , heading);
                telemetry.update();
            }
        }
        resetMove();
    }
    
    public void drop(){ // Drop off the wobble goal.
        goalDeploy.setPosition(1);
        sleep(1000); // 1 second.
        goalClamp.setPosition(0);
        sleep(50); // 0.05 seconds.
        goalDeploy.setPosition(0);
        sleep(1000); // 1 second.
        goalClamp.setPosition(1);
    }
    
    public void feel(){ // Feel amount of rings and change feelState
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
    
    public void resetMove() { // Reset encoder values and stop wheels
        leftBackDrive.setPower(0); // Zero them out
        rightBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        sleep(100); // Pause a bit
        launcherLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the encoders.
        launcherRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
