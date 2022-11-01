package OceanCrashLinearOpMode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.google.gson.interceptors.JsonPostDeserializer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
                .lineToLinearHeading(new Pose2d(-23, 31.5, Math.toRadians(-45)))
                .waitSeconds(3)
                //drop && put lift down
                .lineToLinearHeading(new Pose2d(-23, 31.5, Math.toRadians(90)))
                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(-24, 68, Math.toRadians(90)))
                .waitSeconds(2)
                .turn(-45)
                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(-23, 31.5, Math.toRadians(-45)))
                .waitSeconds(2)
                .build();

        while(!isStarted()){
            pos = vision.getPark();
            telemetry.addData("park: ", pos);
            telemetry.update();
        }

        waitForStart();

        if (!isStopRequested())
        {
            lift.grab();
            drive.followTrajectorySequence(test);
        }
    }
}
