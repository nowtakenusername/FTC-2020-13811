// Directory
package org.firstinspires.ftc.teamcode;

// Imports

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="worstCaseScenario", group="Linear Opmode")
@Disabled
public class worstCaseScenario extends LinearOpMode {

    // Initial creation of objects for motors and servos. They are assigned to their ports below.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftBackDrive, rightBackDrive, rightFrontDrive, leftFrontDrive;

    public void runOpMode() {

//******************************************************************************

        // Initial setup
        // Assign the objects created above to their ports. Hub 1 is the left one.
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive"); //port 0, hub2
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive"); //port 1, hub2
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive"); //port 0, hub1
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive"); //port !, hub1
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

        telemetry.addData("Status", "We're doomed...");
        telemetry.update(); // Done initalizing.

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

//******************************************************************************

//            // Drive controls \\
//            driveX = gamepad1.left_stick_y * speed; // Multiply our inputs by the ideal speed. Defaulted to half power.
//            driveZ = gamepad1.left_stick_x * speed;
//            driveTurn = -gamepad1.right_stick_x * 0.75;
//
//            // This is where we set our drive motors to move diagonally. The robot can move in normal directions, but move the left stick to the side...
//            // and it moves where you want it to. This took considerable time to set up, but it is neatly condensed into four lines.
//            leftBackDrive.setPower((driveX - driveZ) - driveTurn);
//            rightBackDrive.setPower((driveX + driveZ) + driveTurn);
//            leftFrontDrive.setPower((driveX + driveZ) - driveTurn);
//            rightFrontDrive.setPower((driveX - driveZ) + driveTurn);
//
//            // Speed controls \\
//            // This is a simple delay between inputs, so we do not increase or decrease it too fast.
//            if(gamepad1.left_bumper && speed < 1 && speedTimer < runtime.seconds()) {
//                speed+=0.05; speedTimer = runtime.seconds() + 0.5;
//            }
//            if(gamepad1.right_bumper && speed > 0.1 && speedTimer < runtime.seconds()) {
//                speed-=0.05; speedTimer = runtime.seconds() + 0.5;
//            }

//*****************************************************************************

            // Grabber controls \\
            leftBackDrive.setPower(gamepad2.left_stick_y);
            rightBackDrive.setPower(gamepad2.left_stick_y);

//******************************************************************************
//
//            // Launcher controls \\
//            // This controls the feeding conveyor...
//            if(gamepad2.right_trigger > 0.1) {
//                conveyorDrive.setPower(gamepad2.right_trigger);
//            }
//            else if(gamepad2.left_trigger > 0.1) {
//                conveyorDrive.setPower(-gamepad2.left_trigger);
//            }
//            else conveyorDrive.setPower(0);
//
//            // And this controls the launcher.
//            launcherLeft.setPower(gamepad2.right_stick_y);
//            launcherLeft.setPower(gamepad2.right_stick_y);
//
////******************************************************************************
//
//            // Intake controls \\
//            if(gamepad2.right_stick_y < -0.1) {
//                intakeLeft.setPosition(0);
//                intakeRight.setPosition(1);
//            }
//            if(gamepad2.right_stick_y > 0.1) {
//                intakeLeft.setPosition(1);
//                intakeRight.setPosition(0);
//            }

//******************************************************************************

            // Telemetry display \\
            telemetry.addData("Run Time:", "" + runtime.toString());
            telemetry.addData("Motor Power", "LR (%.2f), FB (%.2f)", gamepad1.left_stick_x, gamepad1.left_stick_y);
            telemetry.addData("Launcher Power ", gamepad2.right_trigger);
            telemetry.update();
        }
    }
}
    