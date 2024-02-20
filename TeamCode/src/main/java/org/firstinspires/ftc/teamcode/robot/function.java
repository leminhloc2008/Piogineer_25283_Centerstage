package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;

@TeleOp(name="Test")

public class function extends OpMode {

    DcMotor slider, slider2, intake;
    double target, error, last_error, curr, speed;
    double P = 0.5, I = 0, D = 100;
    double kp, ki, kd;
    boolean first_time_slider = true, first_time_intake = true, intakeToggle = false, sliderToggle = false;
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();


    @Override
    public void init() {
        slider = hardwareMap.dcMotor.get("slider");
        slider2 = hardwareMap.dcMotor.get("slider2");
        intake = hardwareMap.dcMotor.get("intake");
        slider2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        kp = 0.5; ki = 0; kd = 100;
    }

    public double PID(double target)
    {
        curr = (slider.getCurrentPosition() + slider2.getCurrentPosition())/2;
        telemetry.addData("Current pos: ", curr);
        error = target - curr;
        last_error = error;

        P = error;
        I = error + I;
        D = error - last_error;

        speed = P * kp + I * ki + D * kd;
        speed = speed/400;
        return speed;
    }

    @Override
    public void loop() {
        telemetry.update();
        telemetry.addData("Speed",speed);

        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        if (gamepad1.a)
        {
            first_time_slider = false;
        }
        if (gamepad1.x)
        {
            first_time_intake = false;
        }

        if (!first_time_slider)
        {
            if (currentGamepad1.a && !previousGamepad1.a)
            {
                sliderToggle = !sliderToggle;
            }
            if (sliderToggle)
            {
                telemetry.addData("suy", "1");
                target = 1250;

            }
            else
            {
                telemetry.addData("suy", "2");
                target = -200;
            }
            double power = PID(target);
            slider.setPower(power);
            slider2.setPower(power);
        }

        if (!first_time_intake)
        {
            if (currentGamepad1.x && !previousGamepad1.x)
            {
                intakeToggle = !intakeToggle;
            }
            if (intakeToggle)
            {
                intake.setPower(1);

            }
            else
            {
                intake.setPower(0);
            }
        }

//        slider.setPower(-0.7);

    }

}
