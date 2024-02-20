package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="Drive Train")
public class DriveTrain extends OpMode {
    public DcMotor leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor;
    public IMU imu;
    public double ss = 1.1;
    double DrivePower = 1.0, Multiplyer = 3;
    public SampleMecanumDrive drive;
    public static double turnMultiplier = .3, speedMultiplier = .2, strafeMultiplier = .2;
    DcMotor slider, slider2, intake;
    DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    DcMotor keo1, keo2;


    boolean first_time_servo = true, servoToggle = false;

    double target1, target2, error, last_error, curr, speed;

    int sliderToggle = 0;
    double P = 0.5, I = 0, D = 100;
    double kp, ki, kd;
    boolean first_time_slider = true, first_time_intake = true, intakeToggle = false, hopToggle = false, first_time_hop = true, first_time_open = true;
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();

    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    ServoImplEx hop1, hop2, open_up;

    @Override
    public void init() {
        leftFrontMotor = hardwareMap.get(DcMotor.class, "lf");
        leftBackMotor = hardwareMap.get(DcMotor.class, "lb");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rf");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rb");

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        hop1 = hardwareMap.get(ServoImplEx.class,"hop1");
        hop2 = hardwareMap.get(ServoImplEx.class, "hop2");
        open_up = hardwareMap.get(ServoImplEx.class, "open_up");

        hop1.setPwmRange(new PwmControl.PwmRange(500,2500));
        hop2.setPwmRange(new PwmControl.PwmRange(500, 2500));
        open_up.setPwmRange(new PwmControl.PwmRange(500,2500));

        slider = hardwareMap.dcMotor.get("slider");
        slider2 = hardwareMap.dcMotor.get("slider2");
        intake = hardwareMap.dcMotor.get("intake");

        slider2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hop2.setDirection(Servo.Direction.REVERSE);


        kp = 0.5; ki = 0; kd = 100;

        drive = new SampleMecanumDrive(hardwareMap);
    }

    public double PID(double target, double curr)
    {
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


    public void power(double output){
        leftFrontMotor.setPower(-output);
        leftBackMotor.setPower(-output);
        rightFrontMotor.setPower(output);
        rightBackMotor.setPower(output);
    }

    private void moveRobot(double leftStickY, double leftStickX, double rightStickX){
        if (gamepad1.options) imu.resetYaw();

        double directionHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotateX = leftStickX * Math.cos(-directionHeading) - leftStickY * Math.sin(-directionHeading);
        double rotateY = leftStickX * Math.sin(-directionHeading) + leftStickY * Math.cos(-directionHeading);

        rotateX *= ss;

        double denominator = Math.max(Math.abs(rotateY) + Math.abs(rotateX) + Math.abs(rightStickX), 1);

        double topLeftPower = (rotateY + rotateX + rightStickX) / denominator;
        double bottomLeftPower = (rotateY - rotateX + rightStickX) / denominator;
        double topRightPower = (rotateY - rotateX - rightStickX) / denominator;
        double bottomRightPower = (rotateY + rotateX - rightStickX) / denominator;

        leftFrontMotor.setPower(topLeftPower);
        rightFrontMotor.setPower(topRightPower);
        leftBackMotor.setPower(bottomLeftPower);
        rightBackMotor.setPower(bottomRightPower);
    }

    private void moveRobotV2 (double translationAngle, double translationPower, double turnPower) {
        double ADPower = translationPower * Math.sqrt(2) * 0.5 * (Math.sin(translationPower) + Math.cos(translationAngle));
        double BCPower = translationPower * Math.sqrt(2) * 0.5 * (Math.sin(translationPower) - Math.cos(translationAngle));

        double turningScale = Math.max(Math.abs(ADPower + turnPower), Math.abs(ADPower - turnPower));
        turningScale = Math.max(turningScale, Math.max(Math.abs(BCPower + turnPower), Math.abs(BCPower - turnPower)));

        if (Math.abs(turningScale) < 1.0) turningScale = 1.0;

        leftFrontMotor.setPower((ADPower - turningScale) / turningScale);
        leftBackMotor.setPower((BCPower - turningScale) / turningScale);
        rightFrontMotor.setPower((BCPower + turningScale) / turningScale);
        rightBackMotor.setPower((ADPower + turningScale) / turningScale);
    }
    // 192.168.43.1:8080/dash
    private void moveRobotV3 (Mecanum.Motion motion) {
        Mecanum.Wheels wheels = Mecanum.motionToWheels(motion);
        leftFrontMotor.setPower(wheels.leftFront * DrivePower);
        rightFrontMotor.setPower(wheels.rightFront * DrivePower);
        leftBackMotor.setPower(wheels.leftBack * DrivePower);
        rightBackMotor.setPower(wheels.rightBack * DrivePower);
    }

    private void moveRobotV4() {
        drive.setWeightedDrivePower(
                new Pose2d(
                    gamepad1.left_stick_y * speedMultiplier,
                    gamepad1.left_stick_x * strafeMultiplier,
                    -gamepad1.right_stick_x * turnMultiplier
                )
        );
    }

    private void moveV5() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftFrontMotor.setPower(frontLeftPower);
        leftBackMotor.setPower(backLeftPower);
        rightFrontMotor.setPower(frontRightPower);
        rightBackMotor.setPower(backRightPower);
    }

    private void moveV6() {
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;
        if(gamepad1.left_stick_y > 0.15 || gamepad1.left_stick_y < -0.15 || gamepad1.left_stick_x > 0.15 || gamepad1.left_stick_x < -0.15) {
            leftFrontMotor.setPower(v1);
            rightFrontMotor.setPower(v2);
            leftBackMotor.setPower(v3);
            rightBackMotor.setPower(v4);
        }
        else if(gamepad1.right_bumper) {
            leftFrontMotor.setPower(0.5);
            leftBackMotor.setPower(0.5);
            rightFrontMotor.setPower(-0.5);
            rightBackMotor.setPower(-0.5);
        }
        else if(gamepad1.left_bumper) {
            leftFrontMotor.setPower(-0.5);
            leftBackMotor.setPower(-0.5);
            rightFrontMotor.setPower(0.5);
            rightBackMotor.setPower(0.5);
        }
        else {
            leftFrontMotor.setPower(0);
            leftBackMotor.setPower(0);
            rightFrontMotor.setPower(0);
            rightBackMotor.setPower(0);
        }
    }

    public void loop() {
        // moveRobot(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        if (gamepad1.right_stick_y > 0) moveV5();
        else moveV6();
        //moveRobotV3(Mecanum.joystickToMotion(-gamepad1.left_stick_y, gamepad1.left_stick_x,
          //      gamepad1.right_stick_x, -gamepad1.right_stick_y));
        //DrivePower = 1 / ( 1 + gamepad1.right_trigger * Multiplyer);

        previousGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);

        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        if (gamepad2.a)
        {
            first_time_slider = false;
        }
        if (gamepad2.x)
        {
            first_time_intake = false;
        }

        if (!first_time_slider)
        {
            if (currentGamepad2.a && !previousGamepad2.a)
            {
                sliderToggle = sliderToggle + 1;
            }
            if (sliderToggle % 3 == 1)
            {
                sliderToggle = 1;
                telemetry.addData("suy", "1");
                target1 = 11040;
                target2 = 1100;

            }
            else if (sliderToggle % 3 == 0)
            {
                sliderToggle = 0;
                telemetry.addData("suy", "2");
                target1 = 0;
                target2 = -100;
            }
            if (sliderToggle % 3 == 0 || sliderToggle % 3 == 1) {
                slider.setPower(PID(target1, slider.getCurrentPosition()));
                slider2.setPower(-PID(target2, -slider2.getCurrentPosition()));
            } else {
                sliderToggle = 2;
                slider.setPower(0);
                slider2.setPower(0);
            }

        }
        telemetry.addData("Slider state",sliderToggle);
        if (!first_time_intake)
        {
            if (currentGamepad2.x && !previousGamepad2.x)
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

        if (gamepad2.b)
        {
            first_time_hop = false;
        }
        if (!first_time_hop)
        {
            if (currentGamepad2.b && !previousGamepad2.b)
            {
                hopToggle = !hopToggle;
            }
            if (hopToggle)
            {
                hop1.setPosition(Range.clip(0,0.8, hop1.getPosition() + .01));
                hop2.setPosition(Range.clip(0,0.8, hop1.getPosition() + .01));
            }
            else
            {
                hop2.setPosition(Range.clip(0,-0.6, hop2.getPosition() - .01));
                hop1.setPosition(Range.clip(0,-0.6, hop1.getPosition() - .01));
                telemetry.addData("Dua","xuong");
            }
        }

        if (gamepad2.y)
        {
            first_time_open = false;
        }
        if (!first_time_open)
        {
            if (currentGamepad2.y && !previousGamepad2.y)
            {
                servoToggle = !servoToggle;
            }
            if (servoToggle)
            {
                open_up.setPosition(1);
            } else {
                open_up.setPosition(0);
            }
        }
//        slider.setPower(-0.7);

    }

}
