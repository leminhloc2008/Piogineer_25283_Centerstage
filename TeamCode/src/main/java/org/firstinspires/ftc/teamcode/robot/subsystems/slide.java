package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.stateMachines.SlideState;

public class slide implements Subsystem {

    public DcMotor leftMotor, rightMotor;
    private int targetPosition = 0;
    public SlideState sm;
    double kP = 0.01, kI = 0, kD = 0;

    public void activateSlide(HardwareMap hardwareMap) {
        leftMotor = hardwareMap.get(DcMotor.class, "leftSlide");
        rightMotor = hardwareMap.get(DcMotor.class, "rightSlide");

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sm = new SlideState(this);
    }

    public void setTargetPosition(int pos) {
        targetPosition = pos;
    }

    public int getTargetPosition() {
        return targetPosition;
    }

    public void setPID(double p, double i, double d) {
        kP = p;
        kI = i;
        kD = d;
    }

    public int getCurrentPosition() {
        return leftMotor.getCurrentPosition();
    }

    public void setPower(double power){
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    public void setManualPower(double power){
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setPower(power);
        leftMotor.setPower(power);
    }

    public void setPIDPower(double scalar) {
        double currentPosition = leftMotor.getCurrentPosition();
        BasicPID pid = new BasicPID(new PIDCoefficients(kP, kI, kD));

        double power = pid.calculate(targetPosition, currentPosition);

        leftMotor.setPower(scalar * power);
        rightMotor.setPower(scalar * power);
    }

    public void setPIDPower() {
        double currentPosition = leftMotor.getCurrentPosition();
        BasicPID pid = new BasicPID(new PIDCoefficients(kP, kI, kD));

        double power = -pid.calculate(targetPosition, currentPosition);

        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    @Override
    public void update() {
        sm.update();
    }
}