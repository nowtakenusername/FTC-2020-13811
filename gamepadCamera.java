// Directory
package org.firstinspires.ftc.teamcode;

// Imports
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="gamepadCamezing LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY = "AUj2CiD/////AAABmeV6JGe2Y0eyu55x5CrVMzBbTlq4TE9brYNkzy7wmVGIjW4OQsDkpkUwg+OBrgCMbyiG6D3ApoYrz++qh7Y1j5ivMkCcefCAAIPg/hII47NQZWs0qZJzAEej3zB6Vqi6N6a+HBlZM85DREpf51+TWWD1IWJvHdyApzfXF1Iw6MyfnTuUlj0xD/FR+hrpI4iqLg7jawP6OlPJNjjIrf8nzKzuHv8eaQivP3PQvTTMxe4MbF8fK82TUBgFCRjSnQWXRepoVSTqbvlNcaUZEB3mKVY5OwplE+q1UBLSodcORYbEpytvGqXEYP2/pPLGDcIZchzyNkDT6Qkbo9anUiIrr/xyPkI4EVhzb9CNYUlTIBGe";


    // Initial creation of objects for motors and servos. They are assigned to their ports below.
    private ElapsedTime runtime = new ElapsedTime();
    public void runOpMode() {

//******************************************************************************

        // Initial setup
        // Variable declaration
        double driveX = 0; // Used for diagonal control, positive is forward
        double driveZ = 0; // Used for diagonal control, positive is right
        double driveTurn = 0; // Used for turning while moving diagonally.
        double speed = 0.5; // Used to adjust the speed of the robot.
        double speedTimer = 0; // Used to ensure the speed doesn't increase exponentially in an instant.

        h.init(hardwareMap, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update(); // Done initalizing.

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

//******************************************************************************

            // Drive controls \\
            driveX = -gamepad1.left_stick_y * speed; // Multiply our inputs by the ideal speed. Defaulted to half power.
            driveZ = gamepad1.left_stick_x * speed;
            driveTurn = -gamepad1.right_stick_x * 0.5;

            // This is where we set our drive motors to move diagonally. The robot can move in normal directions, but move the left stick to the side...
            // and it moves where you want it to. This took considerable time to set up, but it is neatly condensed into four lines.
            h.leftBackDrive.setPower((driveX - driveZ) - driveTurn);
            h.rightBackDrive.setPower((driveX + driveZ) + driveTurn);
            h.leftFrontDrive.setPower((driveX + driveZ) - driveTurn);
            h.rightFrontDrive.setPower((driveX - driveZ) + driveTurn);

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
            if(gamepad2.x) {
                h.grabberLeft.setPosition(0);
                h.grabberRight.setPosition(1);
            }
            if(gamepad2.y) {
                h.grabberLeft.setPosition(1);
                h.grabberRight.setPosition(0);
            }
            if(gamepad2.right_stick_y > 0.1 || gamepad2.right_stick_y < 0.1) {
                h.grabberArm.setPower(-gamepad2.right_stick_y);
            }
//*****************************************************************************

            // Intake controls \\
            if(gamepad2.left_stick_y > 0.1 || gamepad2.left_stick_y < -0.1) {
                h.launcherLeft.setPower(gamepad2.left_stick_y);
            }
            else {
                h.launcherLeft.setPower(0);
            }
            h.conveyorDrive.setPower(-gamepad2.left_trigger);

            // Continuous servo controls... it's not that bad...
            if(gamepad2.dpad_up) h.intakeSpinner.setPower(1);
            else if(gamepad2.dpad_up) h.intakeSpinner.setPower(0);
            else h.intakeSpinner.setPower(0.5);
            //so you think you can code? but first tell me this: what is code? how is it defined? can you code a computer? well yes obviously but how far do we take this? if computers gain sentience can we still program them? and if so is this really a morally sound thing to do? at what point do computers need the same respect as humans? and if they get that respect can we still use computers for regular things like google? or video games? how would the sentient computers feel? if we advance AI too far are we going to regress to an age without computers

//******************************************************************************

            // Launcher controls \\
            //h.launcherLeft.setPower(gamepad2.right_trigger);
            h.launcherRight.setPower(gamepad2.right_trigger*.75);

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
