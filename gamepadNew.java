package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name="gamepadTank", group="Linear Opmode")
//@Disabled
public class gamepadTank extends LinearOpMode {

    //Declaring OpMode members
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftFrontDrive;
    private DcMotor craneExtend; //Used for encoder wheel forwards
    private DcMotor cranePitch;
    private DcMotor craneElevate;
    private DcMotor fakeMotor; //Used for encoder wheel sideways
    private Servo craneGrab;
    private Servo trayGrab;
    private Servo craneGrabAttitude;
    private Servo flipperLeft;
    private Servo flipperRight;

    public void runOpMode() {
        
//******************************************************************************

        //Initial setup
        //Initialize the motors, servos, to their ports. Servos are all on hub1.
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive"); //port 0, hub1
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive"); //port 1, hub1
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive"); //port 2, hub1
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive"); //port 3, hub1
        craneExtend = hardwareMap.get(DcMotor.class, "craneExtend"); //port 0, hub2
        cranePitch = hardwareMap.get(DcMotor.class, "cranePitch"); //port 1, hub2
        craneElevate = hardwareMap.get(DcMotor.class, "craneElevate"); //port 2, hub2
        fakeMotor = hardwareMap.get(DcMotor.class, "fakeMotor"); //port 2, hub3
        craneGrab = hardwareMap.get(Servo.class, "craneGrab"); //port 0, servo
        craneGrabAttitude = hardwareMap.get(Servo.class, "craneGrabAttitude"); //port 2, servo
        trayGrab = hardwareMap.get(Servo.class, "trayGrab"); //port 1, servo
        flipperLeft = hardwareMap.get(Servo.class, "flipperLeft"); //port 3. servo
        flipperRight = hardwareMap.get(Servo.class, "flipperRight"); //port 4, servo
        
        //Setting the motor directions
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);

        //Initializing variables
        
        telemetry.addData("Status", "Initialized");
        telemetry.update(); //Done initalizing

        //Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
//******************************************************************************

            //Drive controls
            if (gamepad1.left_trigger>0 || gamepad1.right_trigger>0) { //Crab
                    leftBackDrive.setPower(gamepad1.left_stick_x*0.8);
                    rightBackDrive.setPower(-gamepad1.left_stick_x*0.8);
                    leftFrontDrive.setPower(-gamepad1.left_stick_x*0.8);
                    rightFrontDrive.setPower(gamepad1.left_stick_x*0.8);
                }

                else  { //Tank
                    leftBackDrive.setPower(-gamepad1.left_stick_x);
                    rightBackDrive.setPower(-gamepad1.right_stick_x);
                    leftFrontDrive.setPower(-gamepad1.left_stick_x);
                    rightFrontDrive.setPower(-gamepad1.right_stick_x);
                }
            }
            
//******************************************************************************

            //Crane control
           
        
//******************************************************************************

            //Telemetry display
            telemetry.addData("Run Time:", "" + runtime.toString());
            telemetry.addData("Motor Power", "L (%.2f), R (%.2f)", gamepad1.left_stick_y, gamepad1.right_stick_y);
            telemetry.update();
        }
    }
}
