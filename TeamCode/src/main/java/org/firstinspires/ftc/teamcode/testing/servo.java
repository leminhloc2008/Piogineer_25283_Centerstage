package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class servo extends OpMode {
    //public CRServo servo;
    public Servo servo;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "servo");
        //servo = hardwareMap.get(CRServo.class, "servo");
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            servo.setPosition(0);
            //servo.setPower(1);
        }
        if (gamepad1.b) {
            servo.setPosition(1);
            //servo.setPower(-1);
        }
        //servo.setPower(0);
    }
}
