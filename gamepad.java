package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="gamepad", group="Linear Opmode")
//@Disabled
public class gamepad extends LinearOpMode {

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
        
        // Testing encoder stuff 
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        
        
        // Assigns the motors to a forward direction. Positive values in the .setPower() are forwards.
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        
        launcherLeft.setDirection(DcMotor.Direction.REVERSE);
        launcherRight.setDirection(DcMotor.Direction.FORWARD);

        // Variable declaration
        double driveX = 0; // Used for diagonal control, positive is forward
        double driveZ = 0; // Used for diagonal control, positive is right
        double driveTurn = 0; // Used for turning controls
        double speed = 0.5;
        double speedTimer = 0;
      
        telemetry.addData("Status", "Initialized");
        telemetry.update(); // Done initalizing

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
//******************************************************************************
            
            // Drive controls \\
            driveX = gamepad1.left_stick_y * speed;
            driveZ = gamepad1.left_stick_x * speed;
            driveTurn = gamepad1.right_stick_x * 0.5;
            
            leftBackDrive.setPower((driveX - driveZ) - driveTurn);
            rightBackDrive.setPower((driveX + driveZ) + driveTurn);
            leftFrontDrive.setPower((driveX + driveZ) - driveTurn);
            rightFrontDrive.setPower((driveX - driveZ) + driveTurn);
            
            // Speed controls \\
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
            if(gamepad2.right_trigger >= 0.1) {
                launcherLeft.setPower(gamepad2.right_trigger);
                launcherRight.setPower(gamepad2.right_trigger);
            }
            if(gamepad2.right_trigger < 0.1) {
                launcherLeft.setPower(0);
                launcherRight.setPower(0);
            }
            
//******************************************************************************
            
            // Servo controls \\
            
            
//******************************************************************************

            // Telemetry display \\
            telemetry.addData("Run Time:", "" + runtime.toString());
            telemetry.addData("Motor Power", "LR (%.2f), FB (%.2f)", gamepad1.left_stick_x, gamepad1.left_stick_y);
            telemetry.addData("Launcher Power ", gamepad2.right_trigger);
            telemetry.update();
        }
    }
}
