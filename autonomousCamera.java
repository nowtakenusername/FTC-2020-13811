   // Directory
package org.firstinspires.ftc.teamcode;

// Imports

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name="autonomousCamera", group="Linear Opmode")
// @Disabled
public class autonomousCamera extends LinearOpMode {
    // Hardware setup
    hardwareSetup h = new hardwareSetup(); //h
    // Camera Setup
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    // Vuforia Key
    private static final String VUFORIA_KEY = "AUj2CiD/////AAABmeV6JGe2Y0eyu55x5CrVMzBbTlq4TE9brYNkzy7wmVGIjW4OQsDkpkUwg+OBrgCMbyiG6D3ApoYrz++qh7Y1j5ivMkCcefCAAIPg/hII47NQZWs0qZJzAEej3zB6Vqi6N6a+HBlZM85DREpf51+TWWD1IWJvHdyApzfXF1Iw6MyfnTuUlj0xD/FR+hrpI4iqLg7jawP6OlPJNjjIrf8nzKzuHv8eaQivP3PQvTTMxe4MbF8fK82TUBgFCRjSnQWXRepoVSTqbvlNcaUZEB3mKVY5OwplE+q1UBLSodcORYbEpytvGqXEYP2/pPLGDcIZchzyNkDT6Qkbo9anUiIrr/xyPkI4EVhzb9CNYUlTIBGe";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    // Declaring OpMode objects...
    private ElapsedTime runtime = new ElapsedTime();
    private BNO055IMU imu;

    // Setting up variables...
    Orientation lastAngles = new Orientation();
    double globalAngle;
    double heading = 0;
    int inchMult = 229;
    public String scanState = "";

    @Override
    public void runOpMode() {
        // Initialize the hardware and motor stuff
        h.init(hardwareMap, telemetry);
        // Initialize imu
        imu = hardwareMap.get(BNO055IMU.class, "imu"); //

        // Setting up imu parameters...
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false; // Don't fill the log up with garbage...

        // Settng up odometry wheels...
        h.grabberArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // We use the launcher rollers as our encoder port donors...
        h.conveyorDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // It's the simplest way we can do it.


        // Set motor directions...
        h.leftBackDrive.setDirection(DcMotor.Direction.REVERSE); // The motor directions are seriously messed up.
        h.rightBackDrive.setDirection(DcMotor.Direction.FORWARD); // Forwards is backwards, and vice versa.
        h.leftFrontDrive.setDirection(DcMotor.Direction.REVERSE); // But, since it works pretty nicely with the invertedness of the gamepad's sticks,
        h.rightFrontDrive.setDirection(DcMotor.Direction.FORWARD); // we keep it like this.
        h.grabberArm.setDirection(DcMotor.Direction.REVERSE);
        h.conveyorDrive.setDirection(DcMotor.Direction.FORWARD);

        // Initialize the imu
        imu.initialize(parameters);
        // and calibrate the gyro...
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();
        while (!isStopRequested() && !imu.isGyroCalibrated()) // Just ensure that it REALLY calibrates...
        {
            sleep(50);
            idle();
        }
        // and done.

        // Initialize the camera
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(2.5, 16.0 / 9.0);
        }

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
        // This is the code that will be used in an ideal scenario where the building team does what they are supposed to.

        grabber("closed");
        move("forward",42 * inchMult, 0.5, 100);
        sleep(1000);
        telemetry.addData("", scanState);
        move("left",10 * inchMult, 0.5, 1);
        grabber("drop");
        move("right", 18 * inchMult, 0.5, 1);
        move("forward", 4 * inchMult, 0.5, 100);
        shoot("mid");
        sleep(500);
        shoot("cycleStart");
        sleep(5000);
        shoot("cycleStop");
        move("right", 18 * inchMult, 0.5, 1.5);
        move("forward", 8 * inchMult, 0.5, 100);

//        move("forward",2 * inchMult, 0.5, 100);
//        move("right",3 * inchMult, 0.5, 100);
//        move("forward",28 * inchMult, 0.5, 100);
//        sleep(1000);
//        scanMedian();
//        telemetry.addData("", scanState);
//        if (scanState.equals("")) { // No donuts :(
//            move("forward", 28 * inchMult, 0.5, 100);
//            move("left",10 * inchMult, 0.5, 100);
//            grabber("drop");
//            move("left", 6 * inchMult, 0.5, 100);
//            move("backward", 12 * inchMult, 0.5, 100);
//            shoot("powerShot");
//            sleep(500);
//            shoot("cycleStart");
//            sleep(5000);
//            shoot("cycleStop");
//            move("forward", 8 * inchMult, 0.5, 100);
//        } else if (scanState.equals("Quad")) { // Four donuts
//            move("forward", 54 * inchMult, 0.5, 100);
//            move("left",10 * inchMult, 0.5, 100);
//            grabber("drop");
//            move("left", 6 * inchMult, 0.5, 100);
//            move("backward", 30 * inchMult, 0.5, 100);
//            shoot("powerShot");
//            sleep(500);
//            shoot("cycleStart");
//            sleep(5000);
//            shoot("cycleStop");
//            move("forward", 8 * inchMult, 0.5, 100);
//        } else { // One donut
//            move("forward", 32 * inchMult, 0.5, 100);
//            move("left",18 * inchMult, 0.5, 100);
//            grabber("drop");
//            move("left", 6 * inchMult, 0.5, 100);
//            move("backward", 16 * inchMult, 0.5, 100);
//            move("right", 8 * inchMult, 0.5, 100);
//            shoot("powerShot");
//            sleep(500);
//            shoot("cycleStart");
//            sleep(5000);
//            shoot("cycleStop");
//            move("forward", 8 * inchMult, 0.5, 100);
//        }
        // Camera software shutdown.
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    public void move(String direction, int pulses, double power, double timeout) {
        // Move [direction] [pulse] pulses at [power] power.
        double moveStart = runtime.seconds(); // Prepare for the timeout feature in the logic to come later.
        if (direction == "forward") { // If we are moving forward...
            h.leftBackDrive.setPower(power); // Yes, forward is indeed backward.
            h.rightBackDrive.setPower(power); // Not anymore :(
            h.leftFrontDrive.setPower(power);
            h.rightFrontDrive.setPower(power);
            while (!isStopRequested() && -pulses < h.grabberArm.getCurrentPosition() && timeout + moveStart > runtime.seconds()) {
                // If we are NOT stopped, and desired pulses are more than what our wheels register, and we have not registered a timeout, then continue.
                if (getAngle() > heading) {
                    h.leftBackDrive.setPower(power + 0.05);
                    h.rightBackDrive.setPower(power - 0.05);
                    h.leftFrontDrive.setPower(power + 0.05);
                    h.rightFrontDrive.setPower(power - 0.05);
                }
                if (getAngle() < heading) {
                    h.leftBackDrive.setPower(power - 0.05);
                    h.rightBackDrive.setPower(power + 0.05);
                    h.leftFrontDrive.setPower(power - 0.05);
                    h.rightFrontDrive.setPower(power + 0.05);
                } else {
                    h.leftBackDrive.setPower(power);
                    h.rightBackDrive.setPower(power);
                    h.leftFrontDrive.setPower(power);
                    h.rightFrontDrive.setPower(power);
                }
                // Forcibly update telemetry in the move.
                telemetry.addData("Encoder 1", h.grabberArm.getCurrentPosition());
                telemetry.addData("Encoder 2", h.conveyorDrive.getCurrentPosition());
                telemetry.addData("Degree", getAngle());
                telemetry.addData("Heading", heading);
                telemetry.update();
            }
        } else if (direction == "backward") { // If we are moving backward...
            h.leftBackDrive.setPower(-power); // Once again, backward is positive power and vice versa...
            h.rightBackDrive.setPower(-power); // and yes, I am too afraid to change it.
            h.leftFrontDrive.setPower(-power);
            h.rightFrontDrive.setPower(-power);
            while (!isStopRequested() && pulses > h.grabberArm.getCurrentPosition() && timeout + moveStart > runtime.seconds()) {
                // If we are NOT stopped, and negative desired pulses are less than what our wheels register, and we have not registered a timeout, then continue.
                if (getAngle() > heading) { // This statement attempts to correct the robot to its heading if it is not already corrected.
                    h.leftBackDrive.setPower(-power + 0.05); // Keep in mind that this will create a sort of wiggling motion as the robot is in motion.
                    h.rightBackDrive.setPower(-power - 0.05); // This is unavoidable unless we use a PID, which is a serious challenge to set up.
                    h.leftFrontDrive.setPower(-power + 0.05);
                    h.rightFrontDrive.setPower(-power - 0.05);
                }
                if (getAngle() < heading) {
                    h.leftBackDrive.setPower(-power - 0.05);
                    h.rightBackDrive.setPower(-power + 0.05);
                    h.leftFrontDrive.setPower(-power - 0.05);
                    h.rightFrontDrive.setPower(-power + 0.05);
                } else { // This else statement, believe it or not, will never execute.
                    h.leftBackDrive.setPower(-power);
                    h.rightBackDrive.setPower(-power);
                    h.leftFrontDrive.setPower(-power);
                    h.rightFrontDrive.setPower(-power);
                }
                // Forcibly update telemetry in the move.
                telemetry.addData("Encoder 1", h.grabberArm.getCurrentPosition());
                telemetry.addData("Encoder 2", h.conveyorDrive.getCurrentPosition());
                telemetry.addData("Degree", getAngle());
                telemetry.addData("Heading", heading);
                telemetry.update();
            }
        } else if (direction == "left") { // If we are moving left...
            h.leftBackDrive.setPower(power); // Set the power like this...
            h.rightBackDrive.setPower(-power);
            h.leftFrontDrive.setPower(-power);
            h.rightFrontDrive.setPower(power);
            while (!isStopRequested() && pulses > h.conveyorDrive.getCurrentPosition() && timeout + moveStart > runtime.seconds()) {
                // Same situation as the backward logic, but sideways. 
                // Making the correction function for this would give me a massive headache, unfortunately.
                telemetry.addData("Encoder 1", h.grabberArm.getCurrentPosition()); // FORCIBLY update telemetry in the move.
                telemetry.addData("Encoder 2", h.conveyorDrive.getCurrentPosition());
                telemetry.addData("Degree", getAngle());
                telemetry.addData("Heading", heading);
                telemetry.update();
            }
        } else if (direction == "right") { // If we are moving right...
            h.leftBackDrive.setPower(-power); // Set the power like this...
            h.rightBackDrive.setPower(power);
            h.leftFrontDrive.setPower(power);
            h.rightFrontDrive.setPower(-power);
            while (!isStopRequested() && -pulses < h.conveyorDrive.getCurrentPosition() && timeout + moveStart > runtime.seconds()) {
                // Same situation as the forward logic, but sideways. 
                telemetry.addData("Encoder 1", h.grabberArm.getCurrentPosition()); // FORCIBLY update telemetry in the move.
                telemetry.addData("Encoder 2", h.conveyorDrive.getCurrentPosition());
                telemetry.addData("Degree", getAngle());
                telemetry.addData("Heading", heading);
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
            h.leftBackDrive.setPower(-power);
            h.rightBackDrive.setPower(power);
            h.leftFrontDrive.setPower(-power);
            h.rightFrontDrive.setPower(power);
            while (!isStopRequested() && getAngle() < heading) { // FORCIBLY update telemetry, but also make sure we don't go on yet.
                telemetry.addData("Encoder 1", h.grabberArm.getCurrentPosition());
                telemetry.addData("Encoder 2", h.conveyorDrive.getCurrentPosition());
                telemetry.addData("Degree", getAngle());
                telemetry.addData("Heading", heading);
                telemetry.update();
            }
        }
        if (direction == "right") { // If we are rotating right...
            heading -= degrees; // Setting a heading for correction purposes
            h.leftBackDrive.setPower(power);
            h.rightBackDrive.setPower(-power);
            h.leftFrontDrive.setPower(power);
            h.rightFrontDrive.setPower(-power);
            while (!isStopRequested() && getAngle() > heading) { // FORCIBLY update telemetry, but also make sure we don't go on yet.
                telemetry.addData("Encoder 1", h.grabberArm.getCurrentPosition());
                telemetry.addData("Encoder 2", h.conveyorDrive.getCurrentPosition());
                telemetry.addData("Degree", getAngle());
                telemetry.addData("Heading", heading);
                telemetry.update();
            }
        }
        resetMove();
    }

    public void shoot(String goal) { // We are shooting rings...
        if (goal == "low") { // Shoot for the low goal. We likely will not use this, ever.
            //And we won't do anything with it!
        } else if (goal == "mid") { // Shoot for the middle goal. We probably won't use this.
            //h.launcherLeft.setPower(-0.5);
            h.launcherRight.setPower(0.65);
        } else if (goal == "high") { // Shoot for high goal. We will try to use this.
            //h.launcherLeft.setPower(-0.8);
            h.launcherRight.setPower(0.8);
            sleep(500); // yeah, I sleep.
        } else if (goal == "powerShot"){ // Shoot for the power shot targets. We probably won't use this.
            //h.launcherLeft.setPower(1);
            h.launcherRight.setPower(1);
        } else if (goal == "cycleStart") { // Begin cycling...
            h.conveyorDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            h.conveyorDrive.setPower(-1);
        } else if (goal == "cycleStop") {  // and stop.
            h.conveyorDrive.setPower(0);
            //h.launcherLeft.setPower(0);
            h.launcherRight.setPower(0);
        }
    }

    public void resetMove() { // Reset the encoder values and stop the wheels.
        h.leftBackDrive.setPower(0); // Zero them out...
        h.rightBackDrive.setPower(0);
        h.leftFrontDrive.setPower(0);
        h.rightFrontDrive.setPower(0);
        sleep(100); // Pause a bit... (0.1 seconds)
        h.grabberArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // and reset the encoders.
        h.conveyorDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void resetAngle() { // Resets angle heading.
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        // Note that I don't know how the above line works. IMUs are tricky business. 
        globalAngle = 0; // Resets our global angle heading. It's like the heading variable, but changes more often.
    }

    private double getAngle() { // Converts the IMUs z-axis heading into a proper format.
        // And by proper format, the IMU normally doesn't see degrees like a compass. It sees them like an bisected inverted compass with ends at 180.
        // NOTE: I did copy this from the FTC website. Don't see how you do this on your own.
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

    private void scan() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    scanState = "" + recognition.getLabel();
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                }
                telemetry.addData("Scan State", scanState);
                telemetry.update();
            }
        }
    }

    public void grabber(String mode) {
        if(mode.equals("open")) {
            h.grabberLeft.setPosition(1);
            h.grabberRight.setPosition(0);
        }
        if(mode.equals("closed")) {
            h.grabberLeft.setPosition(0);
            h.grabberRight.setPosition(1);
        }
        if(mode.equals("drop")) {
            h.grabberArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            h.conveyorDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            h.grabberArm.setPower(1);
            sleep(500);
            h.grabberArm.setPower(0.2);
            sleep(1000);
            h.grabberLeft.setPosition(1);
            h.grabberRight.setPosition(0);
            h.grabberArm.setPower(0);
            sleep(800);
            h.grabberArm.setPower(-1);
            sleep(800);
            h.grabberArm.setPower(0);
        }
        resetMove();
    }

    private void scanMedian() { // Scan ten times. If we get ANYTHING, use it.
        String scanMedian = ""; // Empty string! Don't ask why I didn't just makes this a return method.
        for(int joe = 0; joe > 10; joe++) { // what's joe?
            scan();
            if(scanState.equals("")) { // .equals NULL
                // Nothing.
            }
            if(scanState.equals("Single")) { // there's a ring, maybe
                scanMedian = "Single";
                scanState = scanMedian;
                break; // jump out of this trainwreck code and proceed as normal
            }
            if(scanState.equals("Quad")) { // definitely a few rings there
                scanMedian = "Quad";
                scanState = scanMedian;
                break;
            }
            sleep(20); // The camera needs some downtime...
        }
        scanState = scanMedian;
    }

    // Needed for camera. No clue how it works.
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    // Needed for camera. No clue how it works.
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
} // END