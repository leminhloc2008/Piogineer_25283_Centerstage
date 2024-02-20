package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="CoreHexMotor")
public class corehexmotor extends OpMode {

    DcMotor hexmotor = null;

    @Override
    public void init() {
        hexmotor = hardwareMap.get(DcMotor.class, "hexmotor");
    }

    @Override
    public void loop() {
        hexmotor.setPower(50);
        telemetry.addData("test", "testing");
    }
}