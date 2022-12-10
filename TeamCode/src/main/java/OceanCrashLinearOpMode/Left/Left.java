package OceanCrashLinearOpMode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.google.gson.interceptors.JsonPostDeserializer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.genetics.ElitisticListPopulation;

import java.util.Vector;

import OceanCrashOpMode.OceanCrashTeleOp;
import OceanCrashRoadrunner.drive.DriveConstants;
import OceanCrashRoadrunner.drive.SampleMecanumDrive;
import OceanCrashRoadrunner.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "Left", group = "Test")
public class Left extends LinearOpMode {

    private SampleMecanumDrive drive;
    private Drivetrain drivetrain;
    private Lift lift;
    private Vision vision;
    private Intake intake;

    public static double stall = -.0004;

    private final double turnP45 = .43;
    private final double turnD45 = .40;

    private final double turnP135 = .24;
    private final double turnD135 = .20;

    private final double moveP48 = .442;
    private final double moveP4 = 1;
    private final double moveP20 = .432;


    ElapsedTime deposit = new ElapsedTime();
    ElapsedTime grab = new ElapsedTime();
    ElapsedTime button = new ElapsedTime();

    TrajectorySequence traj1;
    TrajectorySequence park;

    private int pos;
    private double targetPos;
    private double parkPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);
        drivetrain = new Drivetrain(this);
        vision = new Vision(this);
        lift = new Lift(this);
        lift.grab();
        intake = new Intake(this);

        Pose2d startingPose = new Pose2d(-72, 36, 0);

        drive.setPoseEstimate(startingPose);

        traj1 = drive.trajectorySequenceBuilder(startingPose)
                //PRELOAD
                .addTemporalMarker(0, ()-> lift.extendFourBar())
                .addTemporalMarker(0, () -> targetPos = 2700)
                .waitSeconds(.5)
                .lineToLinearHeading(new Pose2d(-28, 34, Math.toRadians(-25)))
                .UNSTABLE_addTemporalMarkerOffset(0, ()->lift.swivelStart())
                .waitSeconds(.5)
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> lift.release())
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(1.75, () -> targetPos = 400)
                .UNSTABLE_addTemporalMarkerOffset(1.75, () -> lift.swivelOut())
                //CYCLE 1
                .splineTo(new Vector2d(-20, 44), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(-20, 46, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(.5, ()-> lift.grab())
                .UNSTABLE_addTemporalMarkerOffset(1, ()-> targetPos = 1500)
                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(-20, 44, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(.2, ()-> targetPos = 2500)
                .UNSTABLE_addTemporalMarkerOffset(.2, ()-> lift.swivelStart())
                .splineTo(new Vector2d(-28, 36), Math.toRadians(25))
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> lift.release())

                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(1.75, () -> targetPos = 600)
                .UNSTABLE_addTemporalMarkerOffset(1.75, () -> lift.swivelOut())
/*
                //CYCLE 2
                .splineTo(new Vector2d(-20, 48), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(-20, 50, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(.5, ()->lift.grab())
                .UNSTABLE_addTemporalMarkerOffset(1, ()->targetPos = 1500)
                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(-20, 48, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(.2, ()->targetPos = 2500)
                .UNSTABLE_addTemporalMarkerOffset(.2, ()-> lift.swivelStart())
                .splineTo(new Vector2d(-28, 36), Math.toRadians(25))
                .UNSTABLE_addTemporalMarkerOffset(1, ()-> lift.release())
                .UNSTABLE_addTemporalMarkerOffset(2.75, () -> targetPos = 700)
                .UNSTABLE_addTemporalMarkerOffset(2.75, () -> lift.swivelOut())
                .waitSeconds(2)

                //CYCLE 3
                .splineTo(new Vector2d(-20, 48), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(-20, 50, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(.5, ()->lift.grab())
                .UNSTABLE_addTemporalMarkerOffset(1, ()->targetPos = 1500)
                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(-20, 48, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(.2, ()->targetPos = 2500)
                .UNSTABLE_addTemporalMarkerOffset(.2, ()-> lift.swivelStart())
                .splineTo(new Vector2d(-28, 36), Math.toRadians(25))
                .UNSTABLE_addTemporalMarkerOffset(1, ()-> lift.release())
                .UNSTABLE_addTemporalMarkerOffset(2.75, () -> targetPos = 700)
                .UNSTABLE_addTemporalMarkerOffset(2.75, () -> lift.swivelOut())
                .waitSeconds(2)

                //CYCLE 4
                .splineTo(new Vector2d(-20, 48), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(-20, 50, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(.5, ()->lift.grab())
                .UNSTABLE_addTemporalMarkerOffset(1, ()->targetPos = 1500)
                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(-20, 48, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(.2, ()->targetPos = 2500)
                .UNSTABLE_addTemporalMarkerOffset(.2, ()-> lift.swivelStart())
                .splineTo(new Vector2d(-28, 36), Math.toRadians(25))
                .UNSTABLE_addTemporalMarkerOffset(1, ()-> lift.release())
                .UNSTABLE_addTemporalMarkerOffset(2.75, () -> targetPos = 700)
                .UNSTABLE_addTemporalMarkerOffset(2.75, () -> lift.swivelOut())
                .waitSeconds(2)

                //CYCLE 5
                .splineTo(new Vector2d(-20, 48), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(-20, 50, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(.5, ()->lift.grab())
                .UNSTABLE_addTemporalMarkerOffset(1, ()->targetPos = 1500)
                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(-20, 48, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(.2, ()->targetPos = 2500)
                .UNSTABLE_addTemporalMarkerOffset(.2, ()-> lift.swivelStart())
                .splineTo(new Vector2d(-28, 36), Math.toRadians(25))
                .UNSTABLE_addTemporalMarkerOffset(1, ()-> lift.release())
                .UNSTABLE_addTemporalMarkerOffset(2.75, () -> targetPos = 700)
                .UNSTABLE_addTemporalMarkerOffset(2.75, () -> lift.swivelOut())
                .waitSeconds(2)


                //PARK
                .splineTo(new Vector2d(-36, 36), Math.toRadians(90))
                .addDisplacementMarker(() -> drive.followTrajectorySequenceAsync(park))
                */
                .build();

        while(!isStarted()){
            pos = vision.getPark();
            telemetry.addData("park: ", pos);
            telemetry.update();
        }

        switch (pos) {
            case 1:
                parkPos += 21;
                break;
            case 3:
                parkPos -= 21;
                break;
        }
/*
        park = drive.trajectorySequenceBuilder(traj1.end())
                .addTemporalMarker(() -> lift.setLiftPos(0))
                .forward(parkPos)
                .build();
        */

        drive.followTrajectorySequenceAsync(traj1);
        waitForStart();

        while (!isStopRequested())
        {
            drive.update();
            lift.setLiftPos(targetPos);
        }
    }
}
