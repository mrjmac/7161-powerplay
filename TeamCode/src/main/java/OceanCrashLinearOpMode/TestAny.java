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

        Pose2d startingPose = new Pose2d(-21.8, 30.8, 25);

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
                .splineToConstantHeading(new Vector2d(-18, 24), Math.toRadians(-90), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-18, 8), Math.toRadians(245), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(-30, 8, Math.toRadians(0)), Math.toRadians(147.5))
                .build();

        //drive.followTrajectoryAsync(auto);
        waitForStart();
        liftTime.reset();
        while (!isStopRequested())
        {
//------------------------------------DEBUG LIFT----------------------------------------------------
            /*if (liftTime.milliseconds() < 5000)
                lift.setSlideTarget(950);
            else
                lift.setSlideTarget(200);
            lift.updateLiftLength(liftTime.milliseconds());
            telemetry.addData("slidetarget:", lift.currentTargetSlidesPos);
            telemetry.addData("slidepos:", lift.getLiftPos());
            telemetry.update();

             */
//------------------------------------REPLACE CLAW------------------------------------------x----------

            lift.swivelOut();
            lift.release();

             /*

//------------------------------------REPLACE FOUR BAR SERVO----------------------------------------------------

            lift.grab();
            telemetry.addData("retracting:", "yea");
            telemetry.update();
            sleep(1000);
            lift.release();
            telemetry.addData("extending:", "yea");
            telemetry.update();
            sleep(1000);



//-----------------------------------TEST MOVEMENT-----------------------------------------------------
            //drive.update();
            
              */
        }
    }
}
