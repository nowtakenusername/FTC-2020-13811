package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="ReichenMovesRobot", group="Linear Opmode")
//@Disabled
public class ReichenMovesRobot extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftBack, rightBack, rightFront, leftFront;

    public void runOpMode() {
        leftBack  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBack = hardwareMap.get(DcMotor.class, "right_back_drive");
        leftFront = hardwareMap.get(DcMotor.class, "left_front_drive"); 
        rightFront = hardwareMap.get(DcMotor.class, "right_front_drive"); 
        
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        
        waitForStart();
        runtime.reset();
        
        while (opModeIsActive()) {
            if(gamepad1.left_trigger > 0 && gamepad1.right_trigger > 0){
                if(gamepad1.left_stick_y > 0.1){
                    leftBack.setPower(-.5);
                    rightBack.setPower(-.5);
                    rightFront.setPower(-.5);
                    leftFront.setPower(-.5);
                }else
                if(gamepad1.left_stick_y < -0.1){
                    leftBack.setPower(.5);
                    rightBack.setPower(.5);
                    rightFront.setPower(.5);
                    leftFront.setPower(.5);
                }else
                if(gamepad1.right_stick_x < -0.1){
                    leftBack.setPower(-.5);
                    rightBack.setPower(.5);
                    rightFront.setPower(.5);
                    leftFront.setPower(-.5);
                }else
                if(gamepad1.right_stick_x > 0.1){
                    leftBack.setPower(.5);
                    rightBack.setPower(-.5);
                    rightFront.setPower(-.5);
                    leftFront.setPower(.5);
                }else
                if(gamepad1.right_stick_x == 0){
                    leftBack.setPower(0);
                    rightBack.setPower(0);
                    rightFront.setPower(0);
                    leftFront.setPower(0);
                }else
                if(gamepad1.left_stick_y == 0){
                    leftBack.setPower(0);
                    rightBack.setPower(0);
                    rightFront.setPower(0);
                    leftFront.setPower(0);
                }
            }else
            if(gamepad1.left_stick_y > 0.1){
                leftBack.setPower(-.5);
                rightBack.setPower(-.5);
                rightFront.setPower(-.5);
                leftFront.setPower(-.5);
            }else
            if(gamepad1.left_stick_y < -0.1){
                leftBack.setPower(.5);
                rightBack.setPower(.5);
                rightFront.setPower(.5);
                leftFront.setPower(.5);
            }else
            if(gamepad1.right_stick_x < -0.1){
                leftBack.setPower(-.5);
                rightBack.setPower(.5);
                rightFront.setPower(-.5);
                leftFront.setPower(.5);
            }else
            if(gamepad1.right_stick_x > 0.1){
                leftBack.setPower(.5);
                rightBack.setPower(-.5);
                rightFront.setPower(.5);
                leftFront.setPower(-.5);
            }else
            if(gamepad1.right_stick_x == 0){
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);
                leftFront.setPower(0);
            }else
            if(gamepad1.left_stick_y == 0){
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);
                leftFront.setPower(0);
            }
            
        }
    }
}
