package OceanCrashLinearOpMode.Left;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Vector;

import OceanCrashLinearOpMode.Drivetrain;
import OceanCrashLinearOpMode.Intake;
import OceanCrashLinearOpMode.Lift;
import OceanCrashLinearOpMode.Vision;
import OceanCrashRoadrunner.drive.DriveConstants;
import OceanCrashRoadrunner.drive.SampleMecanumDrive;
import OceanCrashRoadrunner.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "LeftEditedSpeed", group = "Left")
public class LeftEditedSpeed extends LinearOpMode {

    private SampleMecanumDrive drive;
    private Drivetrain drivetrain;
    private Lift lift;
    private Vision vision;
    private Intake intake;

    private Trajectory preload, firstGrab, deposit, grab, park, firstDeposit;

    enum State {
        preload,
        drop,
        firstGrab,
        deposit,
        grabCone,
        park
    }

    private State auto = State.preload;

    private int pos;
    private double targetPos;
    private double parkPos = 38;

    ElapsedTime liftTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);
        drivetrain = new Drivetrain(this);
        vision = new Vision(this);
        lift = new Lift(this);
        lift.grab();
        intake = new Intake(this);

        Pose2d startingPose = new Pose2d(-72, 35.3, 0);

        drive.setPoseEstimate(startingPose);

        while(!isStarted()){
            pos = vision.getParkLeft();
            telemetry.addData("park: ", pos);
            telemetry.update();
        }

        switch (pos) {
            case 1:
                parkPos += 15.5;
                break;
            case 2:
                parkPos -= 6;
                break;
            case 3:
                parkPos -= 29;
                break;
        }

        preload = drive.trajectoryBuilder(startingPose)
                .addTemporalMarker(0, ()-> lift.extendFourBar())
                .addTemporalMarker(0, () -> lift.setSlideTarget(875))
                .addTemporalMarker(2.1, ()-> lift.swivelStartLeft())
                .addTemporalMarker(2.4,()->lift.release())
                //.splineToSplineHeading(new Pose2d(-44.3, 34.3, Math.toRadians(0)), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(30, 35, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(-24.0, 34.4, Math.toRadians(-25)), Math.toRadians(-15), SampleMecanumDrive.getVelocityConstraint(25, 35, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(30))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(firstGrab))
                .build();

        firstGrab = drive.trajectoryBuilder(preload.end())
                .splineToSplineHeading(new Pose2d(-21.1, 34.3, Math.toRadians(90)), Math.toRadians(90), SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(-21.1, 49.2, Math.toRadians(90)), Math.toRadians(90),SampleMecanumDrive.getVelocityConstraint(15, Math.toRadians(10), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(firstDeposit))
                .build();

        firstDeposit = drive.trajectoryBuilder(firstGrab.end())
                .lineToLinearHeading(new Pose2d(-28.3, 33.1, Math.toRadians(-25)), SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(70), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(grab))
                .build();

        grab = drive.trajectoryBuilder(firstDeposit.end())
                .splineToSplineHeading(new Pose2d(-21.1, 34.3, Math.toRadians(90)), Math.toRadians(90), SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(-21.1, 49.2, Math.toRadians(90)), Math.toRadians(90),SampleMecanumDrive.getVelocityConstraint(15, Math.toRadians(10), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(deposit))
                .build();

        deposit = drive.trajectoryBuilder(grab.end())
                .lineToLinearHeading(new Pose2d(-28.3, 33.1, Math.toRadians(-25)), SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(70), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //.addDisplacementMarker(() -> drive.followTrajectoryAsync(grab))
                .build();



        /*
        park = drive.trajectoryBuilder(deposit.end())
                .build();
         */

        waitForStart();

        liftTime.reset();
        drive.followTrajectoryAsync(preload);

        while (!isStopRequested())
        {
            drive.update();
            lift.updateLiftLength(liftTime.milliseconds());
            if (!drive.isBusy() && lift.getLiftPos() < 100) break;
        }
    }
}