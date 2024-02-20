package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="Drive Test v2")

public class ArcadeDrive extends LinearOpMode {
    public DcMotor leftMotor, rightMotor, hexMotor;
    public double leftPower, rightPower, xValue, yValue, globalAngle, power = 0.30, correction, rotation;
    public boolean curr = false, last = false;
    public BNO055IMU imu;
    public Orientation lastAngle = new Orientation();
    public PIDController pidRotate, pidDrive;

    @Override
    public void runOpMode() throws InterruptedException  {
        leftMotor = hardwareMap.dcMotor.get("leftWheel");
        rightMotor = hardwareMap.dcMotor.get("rightWheel");

        hexMotor = hardwareMap.get(DcMotor.class, "motor1");

        leftMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        pidRotate = new PIDController(0.003, 0.00003, 0);
        pidDrive = new PIDController(0.05, 0, 0);

        telemetry.addData("Calibrating", "....");
        telemetry.update();

        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Bam bat dau", "long bel");
        telemetry.update();

        waitForStart();

        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();

        while (opModeIsActive())
        {
            correction = pidDrive.performPID(getAngle());
            yValue = gamepad1.right_stick_y * -1;
            xValue = gamepad1.right_stick_x * -1;

            leftPower =  yValue - xValue;
            rightPower = yValue + xValue;

            leftMotor.setPower(Range.clip(leftPower - correction, -1.0, 1.0));
            rightMotor.setPower(Range.clip(rightPower + correction, -1.0, 1.0));

            curr = gamepad1.y;
            if (curr && !last) {
                hexMotor.setPower(-1);
            }
            else hexMotor.setPower(0);
            last = curr;

            idle();
        }
    }

    private void resetAngle() {
        lastAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    private double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngle.firstAngle;

        if (deltaAngle < -180) {
            deltaAngle += 360;
        }
        else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }

        globalAngle += deltaAngle;
        lastAngle = angles;

        return globalAngle;
    }

    private void rotate (int degrees, double power) {
        resetAngle();

        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        if (degrees < 0) {
            while (opModeIsActive() && getAngle() == 0) {
                leftMotor.setPower(power);
                rightMotor.setPower(-power);
                sleep(100);
            }

            do {
                power = pidRotate.performPID(getAngle());
                leftMotor.setPower(-power);
                rightMotor.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());
        }
        else {
            do {
                power = pidRotate.performPID(getAngle());
                leftMotor.setPower(-power);
                rightMotor.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);

        rotation = getAngle();
        sleep(500);
        resetAngle();
    }
}
