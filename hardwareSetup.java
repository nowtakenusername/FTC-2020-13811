package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class hardwareSetup {
    // Initial creation of objects for motors and servos. They are assigned to their ports below.
    public DcMotor leftBackDrive, rightBackDrive, rightFrontDrive, leftFrontDrive, // In order from 0 to 3 on the Control Hub
                   launcherLeft, launcherRight, conveyorDrive, grabberArm; // In order from 0 to 3 on the Expansion Hub
    public Servo grabberLeft, grabberRight, intakeArm; // In order from 0 to 3 on the Control Hub
    public CRServo intakeSpinner; // It's on port 4.

    private ElapsedTime runtime = new ElapsedTime();
    Telemetry telemetry;
    HardwareMap hard; // haha, get it? doing this is HARD.
    public static final int pulsePerInch = 229;

    public hardwareSetup() {
        // huh?
    }

    public void init(HardwareMap tempMap, Telemetry tempTelemetry){ // yeah, I'm no coward, I init EVERYTHING AT ONCE.
        hard = tempMap;
        telemetry = tempTelemetry;

        leftBackDrive  = hard.dcMotor.get("left_back_drive");
        rightBackDrive = hard.dcMotor.get("right_back_drive");
        leftFrontDrive = hard.dcMotor.get("left_front_drive");
        rightFrontDrive = hard.dcMotor.get("right_front_drive");
        launcherLeft = hard.dcMotor.get("launcherLeft");
        launcherRight = hard.dcMotor.get("launcherRight");;
        conveyorDrive = hard.dcMotor.get("conveyorDrive");
        grabberArm = hard.dcMotor.get("grabberArm");
        grabberLeft = hard.servo.get("grabberLeft");
        grabberRight = hard.servo.get("grabberRight");
        intakeArm = hard.servo.get("intakeArm");
        intakeSpinner = hard.crservo.get("intakeSpinner");

        grabberArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Legacy code. This block is not needed, except when the robot spontaneously decides to reset itself. It has happened...
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        grabberArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        conveyorDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Assigns the motors to an inverted direction. Positive values in the .setPower() are backward. Really.
        // The reasoning behind this is simple - pay attention to the inputs, and your code won't be written entirely backwards.
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        // Set the launchers to the right direction. This one is intentionally written backwards.
        // launcherLeft.setDirection(DcMotor.Direction.FORWARD);
        // launcherRight.setDirection(DcMotor.Direction.FORWARD);
    }
}
