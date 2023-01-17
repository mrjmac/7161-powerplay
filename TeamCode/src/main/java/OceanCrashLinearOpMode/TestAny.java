package OceanCrashLinearOpMode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    ElapsedTime liftTime = new ElapsedTime();

    Trajectory auto, auto2;

    private int pos;
    private double targetPos;
    private double parkPos = 34;

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

        while(!isStarted()){
            pos = vision.getParkLeft();
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

        auto = drive.trajectoryBuilder(startingPose)
                .splineToSplineHeading(new Pose2d(-44.3, 34.3, Math.toRadians(0)), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(55, 50, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(-26.8, 32.3, Math.toRadians(-25)), Math.toRadians(-25), SampleMecanumDrive.getVelocityConstraint(55, 50, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //.addDisplacementMarker(() -> drive.followTrajectoryAsync(auto2))
                .build();

        auto2 = drive.trajectoryBuilder(auto.end())
                .splineTo(new Vector2d(5, 5), 0)
                .build();

        drive.followTrajectoryAsync(auto);
        waitForStart();

        while (!isStopRequested())
        {
            drive.update();
            lift.updateLiftLength(liftTime.milliseconds());
            telemetry.addData("slidetarget:", lift.currentTargetSlidesPos);
            telemetry.addData("slidepos:", lift.getLiftPos());
            telemetry.update();
        }
    }
}
