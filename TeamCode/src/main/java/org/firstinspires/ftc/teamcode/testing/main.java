package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

public class main extends OpMode {

    DcMotor motor;

    @Override
    public void init() {

        motor = hardwareMap.get(DcMotor.class, "motor");
        telemetry.addData("Hardware: ", "Initialized");

    }

    @Override
    public void loop() {

        float x = gamepad1.left_stick_x;
        if (x > 0) {
            motor.setPower(x);
        }
        motor.setPower(0);
    }
}