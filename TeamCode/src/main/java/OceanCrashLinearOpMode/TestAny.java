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
@Autonomous(name = "Test Any", group = "Test")
public class TestAny extends LinearOpMode {

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
    Trajectory traj2;

    private int pos;
    private double targetPos;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);
        drivetrain = new Drivetrain(this);
        vision = new Vision(this);
        lift = new Lift(this);
        intake = new Intake(this);

        Pose2d startingPose = new Pose2d(-72, 36, 0);

        drive.setPoseEstimate(startingPose);

        traj1 = drive.trajectorySequenceBuilder(startingPose)
                .addTemporalMarker(0, ()-> lift.extendFourBar())
                .addTemporalMarker(1, ()-> lift.swivelStart())
                .addTemporalMarker(0, () -> targetPos = 2500)
                .lineToLinearHeading(new Pose2d(-28, 36, Math.toRadians(-25)))
                .UNSTABLE_addTemporalMarkerOffset(1, ()-> lift.release())
                .UNSTABLE_addTemporalMarkerOffset(2.75, () -> targetPos = 700)
                .UNSTABLE_addTemporalMarkerOffset(2.75, () -> lift.swivelOut())
                .waitSeconds(2)
                /*
                //1
                .splineTo(new Vector2d(-15, 48), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(-15, 50, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(.5, ()->lift.grab())
                .UNSTABLE_addTemporalMarkerOffset(1, ()->targetPos = 1500)
                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(-15, 48, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(.2, ()->targetPos = 2500)
                .UNSTABLE_addTemporalMarkerOffset(.2, ()-> lift.swivelStart())
                .splineTo(new Vector2d(-28, 36), Math.toRadians(25))
                .UNSTABLE_addTemporalMarkerOffset(1, ()-> lift.release())
                .UNSTABLE_addTemporalMarkerOffset(2.75, () -> targetPos = 700)
                .UNSTABLE_addTemporalMarkerOffset(2.75, () -> lift.swivelOut())
                .waitSeconds(2)
                //2
                .splineTo(new Vector2d(-15, 48), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(-15, 50, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(.5, ()->lift.grab())
                .UNSTABLE_addTemporalMarkerOffset(1, ()->targetPos = 1500)
                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(-15, 48, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(.2, ()->targetPos = 2500)
                .UNSTABLE_addTemporalMarkerOffset(.2, ()-> lift.swivelStart())
                .splineTo(new Vector2d(-28, 36), Math.toRadians(25))
                .UNSTABLE_addTemporalMarkerOffset(1, ()-> lift.release())
                .UNSTABLE_addTemporalMarkerOffset(2.75, () -> targetPos = 700)
                .UNSTABLE_addTemporalMarkerOffset(2.75, () -> lift.swivelOut())
                .waitSeconds(2)
                //3
                .splineTo(new Vector2d(-15, 48), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(-15, 50, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(.5, ()->lift.grab())
                .UNSTABLE_addTemporalMarkerOffset(1, ()->targetPos = 1500)
                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(-15, 48, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(.2, ()->targetPos = 2500)
                .UNSTABLE_addTemporalMarkerOffset(.2, ()-> lift.swivelStart())
                .splineTo(new Vector2d(-28, 36), Math.toRadians(25))
                .UNSTABLE_addTemporalMarkerOffset(1, ()-> lift.release())
                .UNSTABLE_addTemporalMarkerOffset(2.75, () -> targetPos = 700)
                .UNSTABLE_addTemporalMarkerOffset(2.75, () -> lift.swivelOut())
                .waitSeconds(2)
                //4
                .splineTo(new Vector2d(-15, 48), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(-15, 50, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(.5, ()->lift.grab())
                .UNSTABLE_addTemporalMarkerOffset(1, ()->targetPos = 1500)
                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(-15, 48, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(.2, ()->targetPos = 2500)
                .UNSTABLE_addTemporalMarkerOffset(.2, ()-> lift.swivelStart())
                .splineTo(new Vector2d(-28, 36), Math.toRadians(25))
                .UNSTABLE_addTemporalMarkerOffset(1, ()-> lift.release())
                .UNSTABLE_addTemporalMarkerOffset(2.75, () -> targetPos = 700)
                .UNSTABLE_addTemporalMarkerOffset(2.75, () -> lift.swivelOut())
                .waitSeconds(2)
                //5
                .splineTo(new Vector2d(-15, 48), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(-15, 50, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(.5, ()->lift.grab())
                .UNSTABLE_addTemporalMarkerOffset(1, ()->targetPos = 1500)
                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(-15, 48, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(.2, ()->targetPos = 2500)
                .UNSTABLE_addTemporalMarkerOffset(.2, ()-> lift.swivelStart())
                .splineTo(new Vector2d(-28, 36), Math.toRadians(25))
                .UNSTABLE_addTemporalMarkerOffset(1, ()-> lift.release())
                .UNSTABLE_addTemporalMarkerOffset(2.75, () -> targetPos = 700)
                .UNSTABLE_addTemporalMarkerOffset(2.75, () -> lift.swivelOut())
                .waitSeconds(2)

                 */
                .build();



        /*
        while(!isStarted()){
            pos = vision.getPark();

            telemetry.addData("park: ", pos);
            telemetry.update();
        }

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
