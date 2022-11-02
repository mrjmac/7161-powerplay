package OceanCrashLinearOpMode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.google.gson.interceptors.JsonPostDeserializer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

    private int pos;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);
        drivetrain = new Drivetrain(this);
        vision = new Vision(this);
        lift = new Lift(this);
        intake = new Intake(this);

        Pose2d startingPose = new Pose2d(-72, 36, 0);

        drive.setPoseEstimate(startingPose);

        TrajectorySequence test = drive.trajectorySequenceBuilder(startingPose)

                .lineToLinearHeading(new Pose2d(-36, 36, Math.toRadians(0)))
                .waitSeconds(2)
                //.UNSTABLE_addTemporalMarkerOffset(0, () -> lift.setLiftPos(1500))
                .lineToLinearHeading(new Pose2d(-24, 35, Math.toRadians(-47)))
                .waitSeconds(3)
                //drop && put lift down
                .UNSTABLE_addTemporalMarkerOffset(0, () -> lift.extendFourBar())
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> lift.setLiftPos(700))
                .turn(Math.toRadians(137))
                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(-27, 57.75, Math.toRadians(90)), SampleMecanumDrive.getVelocityConstraint(20.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(50))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> lift.release())
                .UNSTABLE_addTemporalMarkerOffset(4, () -> lift.resetLift(.3, 600))
                .UNSTABLE_addTemporalMarkerOffset(5, () -> lift.grab())
                .UNSTABLE_addTemporalMarkerOffset(5.6, () -> lift.setLiftPos(900))
                .waitSeconds(6)
                .lineToLinearHeading(new Pose2d(-24, 17, Math.toRadians(90)))
                .waitSeconds(3)
                .turn(Math.toRadians(-47))
                .waitSeconds(2)

                /*
                //.lineToLinearHeading(new Pose2d(-23, 31.5, Math.toRadians(90)))
                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(-24, 68, Math.toRadians(90)))
                .waitSeconds(2)
                .turn(-45)
                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(-23, 31.5, Math.toRadians(-45)))
                .waitSeconds(2)

                 */
                .build();


        while(!isStarted()){
            pos = vision.getPark();
            telemetry.addData("park: ", pos);
            telemetry.update();
        }
        lift.grab();
        waitForStart();

        if (!isStopRequested())
        {

            drive.followTrajectorySequence(test);
        }
    }
}
