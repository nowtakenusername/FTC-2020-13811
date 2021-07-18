// Directory
package org.firstinspires.ftc.teamcode;

// Imports
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="gamepad", group="Linear Opmode")
//@Disabled
public class gamepad extends LinearOpMode {

    // Initial creation of objects for motors and servos. They are assigned to their ports below.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftBackDrive, rightBackDrive, rightFrontDrive, leftFrontDrive, // In order from 0 to 3 on the Control Hub
            launcherLeft, launcherRight, conveyorDrive, grabberArm; // In order from 0 to 3 on the Expansion Hub
    private Servo grabberLeft, grabberRight, intakeLeft, intakeRight; // In order from 0 to 4 on the Control Hub

    boolean launcherRunning = false;
    boolean can_Fire = false;
    boolean fire = false;

    int targetSpeed;
    int atSpeed;

    Runnable sped;
    Thread spedThread;

    speedCallback scb = new speedCallback() {
        @Override
        public int speed() {
            return targetSpeed;
        }

        @Override
        public boolean Running() {
            return launcherRunning;
        }

        @Override
        public void canFire(boolean canFire, int speed) {
            can_Fire = canFire;
            atSpeed = speed;
        }

        @Override
        public boolean fire() {
            return fire;
        }
    };

    public void runOpMode() {

//******************************************************************************

        sped = new speed(hardwareMap, telemetry, launcherLeft, launcherRight, scb);
        spedThread = new Thread(sped);

        spedThread.start();

        // Initial setup
        // Assign the objects created above to their ports. Hub 1 is the left one.
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        launcherLeft = hardwareMap.get(DcMotor.class, "launcherLeft");
        launcherRight = hardwareMap.get(DcMotor.class, "launcherRight");
        conveyorDrive = hardwareMap.get(DcMotor.class, "conveyorDrive");
        grabberArm = hardwareMap.get(DcMotor.class, "grabberArm");
        grabberLeft = hardwareMap.get(Servo.class, "grabberLeft");
        grabberRight = hardwareMap.get(Servo.class, "grabberRight");
        intakeLeft = hardwareMap.get(Servo.class, "intakeLeft");
        intakeRight = hardwareMap.get(Servo.class, "intakeRight");

        // Legacy code. This block is not needed, except when the robot spontaneously decides to reset itself. It has happened...
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Assigns the motors to an inverted direction. Positive values in the .setPower() are backward. Really.
        // The reasoning behind this is simple - pay attention to the inputs, and your code won't be written entirely backwards.
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        // Set the launchers to the right direction. This one is intentionally written backwards.
//        launcherLeft.setDirection(DcMotor.Direction.FORWARD);
//        launcherRight.setDirection(DcMotor.Direction.FORWARD);

        // Variable declaration
        double driveX = 0; // Used for diagonal control, positive is forward
        double driveZ = 0; // Used for diagonal control, positive is right
        double driveTurn = 0; // Used for turning while moving diagonally.
        double speed = 0.8; // Used to adjust the speed of the robot.
        double speedTimer = 0; // Used to ensure the speed doesn't increase exponentially in an instant.

        telemetry.addData("Status", "Initialized");
        telemetry.update(); // Done initalizing.

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

//******************************************************************************

            // Drive controls \\
            driveX = gamepad1.left_stick_y * speed; // Multiply our inputs by the ideal speed. Defaulted to half power.
            driveZ = gamepad1.left_stick_x * speed;
            driveTurn = -gamepad1.right_stick_x * 0.75;

            // This is where we set our drive motors to move diagonally. The robot can move in normal directions, but move the left stick to the side...
            // and it moves where you want it to. This took considerable time to set up, but it is neatly condensed into four lines.
            leftBackDrive.setPower((driveX - driveZ) - driveTurn);
            rightBackDrive.setPower((driveX + driveZ) + driveTurn);
            leftFrontDrive.setPower((driveX + driveZ) - driveTurn);
            rightFrontDrive.setPower((driveX - driveZ) + driveTurn);

            // Speed controls \\
            // This is a simple delay between inputs, so we do not increase or decrease it too fast.
            if(gamepad1.left_bumper && speed < 1 && speedTimer < runtime.seconds()) {
                speed+=0.05; speedTimer = runtime.seconds() + 0.5;
            }
            if(gamepad1.right_bumper && speed > 0.1 && speedTimer < runtime.seconds()) {
                speed-=0.05; speedTimer = runtime.seconds() + 0.5;
            }

//*****************************************************************************

            // Grabber controls \\
            if(gamepad2.left_stick_y != 0) {
                grabberArm.setPower(gamepad2.left_stick_y);
            } else grabberArm.setPower(0);
            if(gamepad2.right_bumper) {
                grabberLeft.setPosition(0);
                grabberRight.setPosition(1);
            }
            if(gamepad2.left_bumper) {
                grabberLeft.setPosition(1);
                grabberRight.setPosition(0);
            }

//******************************************************************************

            // Launcher controls \\
            // This controls the feeding conveyor...
            if(gamepad2.right_trigger > 0.1) {
                conveyorDrive.setPower(gamepad2.right_trigger);
            }
            else if(gamepad2.left_trigger > 0.1) {
                conveyorDrive.setPower(-gamepad2.left_trigger);
            }
            else conveyorDrive.setPower(0);

            // And this controls the launcher.
            if(gamepad2.right_bumper) {
                scb.speed();
            }
//******************************************************************************

            // Intake controls \\
            if(gamepad2.right_stick_y < -0.1) {
                intakeLeft.setPosition(0);
                intakeRight.setPosition(1);
            }
            if(gamepad2.right_stick_y > 0.1) {
                intakeLeft.setPosition(1);
                intakeRight.setPosition(0);
            }

//******************************************************************************

            // Telemetry display \\
            telemetry.addData("Run Time:", "" + runtime.toString());
            telemetry.addData("Motor Power", "LR (%.2f), FB (%.2f)", gamepad1.left_stick_x, gamepad1.left_stick_y);
            telemetry.addData("Launcher Power ", gamepad2.right_trigger);
            telemetry.update();
        }
    }
}
    