package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class speed implements Runnable {

    HardwareMap hard;
    Telemetry telemeer;
    DcMotor motorLeft;
    DcMotor motorRight;

    speedCallback cb;

    public speed(HardwareMap hardw, Telemetry tellem, DcMotor m1, DcMotor m2, speedCallback speedCB) {
        hard = hardw;
        telemeer = tellem;
        motorLeft = m1;
        motorRight = m2;
        cb = speedCB;

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void run() {
        while(cb.Running()) {
            if(cb.fire()) {


            } else {
                motorLeft.setPower(0);
                motorRight.setPower(0);
            }

        }
    }
}

