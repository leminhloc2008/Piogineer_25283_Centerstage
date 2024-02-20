package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@com.qualcomm.robotcore.eventloop.opmode.Autonomous (name = "Red auto")
public class redAuto extends LinearOpMode {
    OpenCvCamera camera;
    Pipeline pipeline = new Pipeline();

    public static Pose2d toPropPose = new Pose2d(9, 36 - 6, Math.toRadians(180));
    public static double toPropPoseAngle = 225;
    public static Pose2d pixelDropPose = new Pose2d(12, 36 - 6, Math.toRadians(180));
    public static double pixelDropPoseAngle = 180;

    @Override
    public void runOpMode() throws InterruptedException {
        // Camera activation
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id",
                        hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap
                .get(WebcamName.class, "camera"), cameraMonitorViewId);

        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        // Declare hardware (hardwaremap) of function

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Prop x value: ", pipeline.getJunctionPoint().x);
            telemetry.addData("Prop area: ", pipeline.getPropAreaAttr());
            telemetry.update();
        }

        // Prepare for auto

        // Drivetrain activation
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startLocation = new Pose2d(15, -63, Math.toRadians(90));
        drive.setPoseEstimate(startLocation);

        /* -------- RIGHT -------- */

        TrajectorySequence rightTrajectoryPurple = drive.trajectorySequenceBuilder(startLocation)
                .splineToLinearHeading(new Pose2d(26, -39, Math.toRadians(90)), Math.toRadians(90))
                .build();

        TrajectorySequence rightTrajectoryYellow = drive.trajectorySequenceBuilder(rightTrajectoryPurple.end())
                .back(6,
                        SampleMecanumDrive.getVelocityConstraint(6, 142.9, 16.34),
                        SampleMecanumDrive.getAccelerationConstraint(52.48))
                .splineToSplineHeading(new Pose2d(51, -43, Math.toRadians(180)), Math.toRadians(0))
                .build();

        TrajectorySequence rightTrajectoryPark = drive.trajectorySequenceBuilder(rightTrajectoryYellow.end())
                .splineToLinearHeading(new Pose2d(45, -65, Math.toRadians(180)), Math.toRadians(270))
                .build();

        /* -------- MIDDLE -------- */

        TrajectorySequence middleTrajectoryPurple = drive.trajectorySequenceBuilder(startLocation)
                .splineToLinearHeading(new Pose2d(15, -32, Math.toRadians(90)), Math.toRadians(90))
                .build();

        TrajectorySequence middleTrajectoryYellow = drive.trajectorySequenceBuilder(middleTrajectoryPurple.end())
                .back(6,
                        SampleMecanumDrive.getVelocityConstraint(6, 142.9, 16.34),
                        SampleMecanumDrive.getAccelerationConstraint(52.48))
                .splineToSplineHeading(new Pose2d(50, -36, Math.toRadians(180)), Math.toRadians(0))
                .build();

        TrajectorySequence middleTrajectoryPark = drive.trajectorySequenceBuilder(middleTrajectoryYellow.end())
                .splineToLinearHeading(new Pose2d(45, -65, Math.toRadians(180)), Math.toRadians(270))
                .build();


        /* -------- LEFT -------- */

        TrajectorySequence leftTrajectoryPurple = drive.trajectorySequenceBuilder(startLocation)
                .splineToLinearHeading(new Pose2d(6, -33, Math.toRadians(135)), Math.toRadians(180))
                .build();

        TrajectorySequence leftTrajectoryYellow = drive.trajectorySequenceBuilder(leftTrajectoryPurple.end())
                .back(6,
                        SampleMecanumDrive.getVelocityConstraint(6, 142.9, 16.34),
                        SampleMecanumDrive.getAccelerationConstraint(52.48))
                .splineToSplineHeading(new Pose2d(50, -30, Math.toRadians(180)), Math.toRadians(0))
                .build();

        TrajectorySequence leftTrajectoryPark = drive.trajectorySequenceBuilder(leftTrajectoryYellow.end())
                .splineToLinearHeading(new Pose2d(42, -65, Math.toRadians(180)), Math.toRadians(270))
                .build();

        double propX = pipeline.getJunctionPoint().x;
        double propArea = pipeline.getPropAreaAttr();

        camera.stopStreaming();

        waitForStart();

        if (isStopRequested()) return;

        // Pixel placed left
        if (propArea < 10000)
        {
            drive.followTrajectorySequence(leftTrajectoryPurple);

            // Drop pixel
            sleep(500);

            // Raise lift
            sleep(1000);

            // Intake
            // Drive to backboard
            drive.followTrajectorySequence(leftTrajectoryYellow);

            // Score
            sleep(200);
            sleep(200);

            // Park
            drive.followTrajectorySequence(leftTrajectoryPark);

        }
        // Spike mark placed right
        else if (propX > 500)
        {
            drive.followTrajectorySequence(rightTrajectoryPurple);

            // Drop pixel
            sleep(500);

            // Raise lift

            sleep(1000);
            // Intake

            // Drive to backboard
            drive.followTrajectorySequence(rightTrajectoryYellow);

            // Score
            sleep(200);

            // Park
            drive.followTrajectorySequence(rightTrajectoryPark);
        }
        // Spike mark placed center
        else
        {
            drive.followTrajectorySequence(middleTrajectoryPurple);

            // Drop pixel
            sleep(500);

            // Raise lift
            sleep(1000);

            // Start intake and deploy trident

            // Drive to backboard
            drive.followTrajectorySequence(middleTrajectoryYellow);

            // Score
            sleep(200);

            // Park
            drive.followTrajectorySequence(middleTrajectoryPark);
        }

        // Prepare for teleop
    }
}