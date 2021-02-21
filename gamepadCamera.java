// Directory
package org.firstinspires.ftc.teamcode;

// Imports
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="gamepadCamera", group="Linear Opmode")
//@Disabled

public class gamepadCamera extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY = "AUj2CiD/////AAABmeV6JGe2Y0eyu55x5CrVMzBbTlq4TE9brYNkzy7wmVGIjW4OQsDkpkUwg+OBrgCMbyiG6D3ApoYrz++qh7Y1j5ivMkCcefCAAIPg/hII47NQZWs0qZJzAEej3zB6Vqi6N6a+HBlZM85DREpf51+TWWD1IWJvHdyApzfXF1Iw6MyfnTuUlj0xD/FR+hrpI4iqLg7jawP6OlPJNjjIrf8nzKzuHv8eaQivP3PQvTTMxe4MbF8fK82TUBgFCRjSnQWXRepoVSTqbvlNcaUZEB3mKVY5OwplE+q1UBLSodcORYbEpytvGqXEYP2/pPLGDcIZchzyNkDT6Qkbo9anUiIrr/xyPkI4EVhzb9CNYUlTIBGe";


    // Initial creation of objects for motors and servos. They are assigned to their ports below.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftBackDrive, rightBackDrive, rightFrontDrive, leftFrontDrive,
            launcherLeft, launcherRight, launcherElevate,conveyorDrive;
    private Servo grabberLeft, grabberRight, ringFeeler;
    public void runOpMode() {

//******************************************************************************

        // Initial setup
        // Assign the objects created above to their ports. Hub 1 is the left one.
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive"); //port 0, hub2
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive"); //port 1, hub2
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive"); //port 0, hub1
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive"); //port !, hub1
        launcherLeft = hardwareMap.get(DcMotor.class, "launcherLeft");
        launcherRight = hardwareMap.get(DcMotor.class, "launcherRight");
        launcherElevate = hardwareMap.get(DcMotor.class, "launcherElevate");
        conveyorDrive = hardwareMap.get(DcMotor.class, "conveyorDrive");
        grabberLeft = hardwareMap.get(Servo.class, "grabberLeft");
        grabberRight = hardwareMap.get(Servo.class, "grabberRight");
        ringFeeler = hardwareMap.get(Servo.class, "ringFeeler");

        // Legacy code. This block is not needed, except when the robot spontaneously decides to reset itself. It has happened...
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Assigns the motors to an inverted direction. Positive values in the .setPower() are backward. Really.
        // The reasoning behind this is simple - pay attention to the inputs, and your code won't be written entirely backwards.
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        // Set the launchers to the right direction. This one is intentionally written backwards.
        launcherLeft.setDirection(DcMotor.Direction.REVERSE);
        launcherRight.setDirection(DcMotor.Direction.FORWARD);

        // Variable declaration
        double driveX = 0; // Used for diagonal control, positive is forward
        double driveZ = 0; // Used for diagonal control, positive is right
        double driveTurn = 0; // Used for turning while moving diagonally.
        double speed = 0.5; // Used to adjust the speed of the robot.
        double speedTimer = 0; // Used to ensure the speed doesn't increase exponentially in an instant.
        boolean launcherManual = true; // True means we are using the manual launcher spinup mode.
        double launcherSpeed = 0.5; // Used for \'automatedly\' changing the launcher speed.
        double launcherTimer = 0; // Used for ensuring that the speed doesn't increase a hundred times a second.

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
            driveTurn = gamepad1.right_stick_x * 0.5;

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

            // if(gamepad1.x && goalDeploy.getPosition() > 0.5){//toggles wobble goal grabber with X
            //     goalDeploy.setPosition(1);
            //     sleep(1000); // 1 seconds
            //     goalClamp.setPosition(1);
            // }
            // if(gamepad1.x && goalDeploy.getPosition() < 0.5){//toggles wobble goal grabber with X
            //     goalDeploy.setPosition(0);
            //     sleep(1000);
            //     goalClamp.setPosition(0);
            // }

//******************************************************************************

            // Launcher controls \\
            // There are two modes, one manual and the other automated. Manual is used for debugging purposes.
            if(launcherManual == true) { // Using trigger controls.
                if(gamepad2.right_trigger >= 0.1) { // Note that it isn't set to >= 0. Sometimes you have to do things like that for foolproofing.
                    launcherLeft.setPower(gamepad2.right_trigger);
                    launcherRight.setPower(gamepad2.right_trigger);
                }
                if(gamepad2.right_trigger < 0.1) { // and anything else...
                    launcherLeft.setPower(0); // Stop the motors.
                    launcherRight.setPower(0);
                }
                if(gamepad2.left_bumper && gamepad2.right_bumper && launcherTimer > runtime.seconds()) {
                    launcherManual = false;
                    launcherTimer = runtime.seconds() + 0.5;
                }
            }
            if(launcherManual == false) {
                if(gamepad2.left_bumper && gamepad2.right_bumper && launcherTimer > runtime.seconds()) {
                    launcherManual = true;
                    launcherTimer = runtime.seconds() + 0.5;
                }
            }
//******************************************************************************

            // Servo controls \\


//******************************************************************************

            // Telemetry display \\
            telemetry.addData("Run Time:", "" + runtime.toString());
            telemetry.addData("Motor Power", "LR (%.2f), FB (%.2f)", gamepad1.left_stick_x, gamepad1.left_stick_y);
            telemetry.addData("Launcher Power ", gamepad2.right_trigger);
            telemetry.addData("Launcher Power ", gamepad2.right_trigger);
            telemetry.update();
        }
    }
}
