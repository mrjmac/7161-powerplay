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

import OceanCrashOpMode.OceanCrashTeleOp;
import OceanCrashRoadrunner.drive.DriveConstants;
import OceanCrashRoadrunner.drive.SampleMecanumDrive;
import OceanCrashRoadrunner.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "Park", group = "Test")
public class Park extends LinearOpMode {

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
    private int parkMod = 36;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);
        drivetrain = new Drivetrain(this);
        vision = new Vision(this);
        lift = new Lift(this);
        intake = new Intake(this);

        Pose2d startingPose = new Pose2d(-72, 36, 0);

        drive.setPoseEstimate(startingPose);






        while(!isStarted()){
            pos = vision.getParkLeft();

            telemetry.addData("park: ", pos);
            telemetry.update();
        }

        if (pos == 1)
            parkMod += 24;
        else if (pos == 3)
            parkMod -= 24;
        else
            parkMod++;

        traj1 = drive.trajectorySequenceBuilder(startingPose)
                .lineToLinearHeading(new Pose2d(-46, 36, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-46, parkMod))
                .build();

        drive.followTrajectorySequenceAsync(traj1);
        waitForStart();

        while (!isStopRequested())
        {
            drive.update();
            if (!drive.isBusy() && lift.getLiftPos() < 100) break;
        }
    }
}
