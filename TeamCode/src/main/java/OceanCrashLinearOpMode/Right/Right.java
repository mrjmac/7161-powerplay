package OceanCrashLinearOpMode.Right;

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
@Autonomous(name = "Right", group = "Right")
public class Right extends LinearOpMode {

    /*
        TODO:
        TO GREATLY INCREASE CYCLE SPEED, BREAK DEPOSIT UP INTO SPLINES AND SET LIFT LOW ON GRAB, HIGH ONCE WE GET CLOSE
     */

    private SampleMecanumDrive drive;
    private Drivetrain drivetrain;
    private Lift lift;
    private Vision vision;
    private Intake intake;

    private Trajectory preload;

    private Trajectory[] park = new Trajectory[3];
    private Trajectory[] grabs = new Trajectory[5];
    private Trajectory[] deposits = new Trajectory[5];
    private double stop;

    enum State {
        preload,
        grabp,
        grab,
        deposit,
        park
    }

    private State auto = State.preload;
    private boolean goNext = false;

    private int pos, cycleNum = 0;
    private double targetPos;
    private double parkPos = 38;

    ElapsedTime liftTime = new ElapsedTime();
    ElapsedTime state = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);
        drivetrain = new Drivetrain(this);
        vision = new Vision(this);
        lift = new Lift(this);
        lift.grab();
        intake = new Intake(this);

        Pose2d startingPose = new Pose2d(-72, -35.3 + (3.0/8), 0);

        drive.setPoseEstimate(startingPose);



        preload = drive.trajectoryBuilder(startingPose)
                .addTemporalMarker(0, ()-> lift.extendFourBar())
                .addTemporalMarker(1.1, () -> lift.setSlideTarget(850))
                .addTemporalMarker(2.1, ()-> lift.swivelStartRight())
                //.splineToSplineHeading(new Pose2d(-40, 35, Math.toRadians(0)), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(60))
                .splineToSplineHeading(new Pose2d(-24.9, -32.5, Math.toRadians(25)), Math.toRadians(15), SampleMecanumDrive.getVelocityConstraint(33, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL - 5))
                .build();

        grabs[0] = drive.trajectoryBuilder(preload.end())
                .addTemporalMarker(.4, () -> lift.setSlideTarget(125))
                .addTemporalMarker(.7, ()-> lift.trueExtendFourBar())
                .addTemporalMarker(.9, ()-> lift.swivelOut())
                .splineToSplineHeading(new Pose2d(-23.8, -34.3, Math.toRadians(-90)), Math.toRadians(-90), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL - 5))
                .splineToSplineHeading(new Pose2d(-23.8, -48.5, Math.toRadians(-90)), Math.toRadians(-90), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL - 5))
                .build();

        deposits[0] = drive.trajectoryBuilder(grabs[0].end())
                .addTemporalMarker(2, () -> lift.extendFourBar())
                .addTemporalMarker(.25, () -> lift.setSlideTarget(850))
                .lineToLinearHeading(new Pose2d(-28.3, -30.1, Math.toRadians(25)), SampleMecanumDrive.getVelocityConstraint(50, Math.toRadians(60), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(20))
                .build();

        grabs[1] = drive.trajectoryBuilder(deposits[0].end())
                .addTemporalMarker(.4, () -> lift.setSlideTarget(100))
                .addTemporalMarker(.7, ()-> lift.trueExtendFourBar())
                .addTemporalMarker(.9, ()-> lift.swivelOut())
                .splineToSplineHeading(new Pose2d(-23.3, -34.3, Math.toRadians(-90)), Math.toRadians(-90), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL - 5))
                .splineToSplineHeading(new Pose2d(-23.3, -48.1, Math.toRadians(-90)), Math.toRadians(-90), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL - 5))
                .build();

        deposits[1] = drive.trajectoryBuilder(grabs[1].end())
                .addTemporalMarker(2, () -> lift.extendFourBar())
                .addTemporalMarker(.25, () -> lift.setSlideTarget(850))
                .lineToLinearHeading(new Pose2d(-27.1, -29.7, Math.toRadians(25)), SampleMecanumDrive.getVelocityConstraint(50, Math.toRadians(60), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(20))
                .build();

        grabs[2] = drive.trajectoryBuilder(deposits[1].end())
                .addTemporalMarker(.4, () -> lift.setSlideTarget(60))
                .addTemporalMarker(.7, ()-> lift.trueExtendFourBar())
                .addTemporalMarker(.9, ()-> lift.swivelOut())
                .splineToSplineHeading(new Pose2d(-22.5, -34.3, Math.toRadians(-90)), Math.toRadians(-90), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL - 5))
                .splineToSplineHeading(new Pose2d(-22.5, -48.2, Math.toRadians(-90)), Math.toRadians(-90), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL - 5))
                .build();

        deposits[2] = drive.trajectoryBuilder(grabs[2].end())
                .addTemporalMarker(2, () -> lift.extendFourBar())
                .addTemporalMarker(.25, () -> lift.setSlideTarget(850))
                .lineToLinearHeading(new Pose2d(-26.3, -29.5, Math.toRadians(25)), SampleMecanumDrive.getVelocityConstraint(50, Math.toRadians(60), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(20))
                .build();

        grabs[3] = drive.trajectoryBuilder(deposits[2].end())
                .addTemporalMarker(.4, () -> lift.setSlideTarget(25))
                .addTemporalMarker(.7, ()-> lift.trueExtendFourBar())
                .addTemporalMarker(.9, ()-> lift.swivelOut())
                .splineToSplineHeading(new Pose2d(-22.0, -34.3, Math.toRadians(-90)), Math.toRadians(-90), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL - 5))
                .splineToSplineHeading(new Pose2d(-22.0, -47.3, Math.toRadians(-90)), Math.toRadians(-90), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL - 5))
                .build();

        deposits[3] = drive.trajectoryBuilder(grabs[3].end())
                .addTemporalMarker(2, () -> lift.extendFourBar())
                .addTemporalMarker(.25, () -> lift.setSlideTarget(850))
                .lineToLinearHeading(new Pose2d(-27.6, -30.6, Math.toRadians(25)), SampleMecanumDrive.getVelocityConstraint(50, Math.toRadians(60), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(20))
//                                 this thing sucks ^
                .build();

        park[0] = drive.trajectoryBuilder(deposits[3].end(), true)
                .addTemporalMarker(0, () -> lift.grab())
                .addTemporalMarker(.8, ()-> lift.trueExtendFourBar())
                .addTemporalMarker(.8, ()-> lift.swivelIn())
                .addTemporalMarker(2, () -> lift.retractFourBar())
                .addTemporalMarker(.8, ()-> lift.setSlideTarget(0))
                .splineToConstantHeading(new Vector2d(-15, -45), Math.toRadians(-90), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL - 5))
                .splineTo(new Vector2d(-15, -58), Math.toRadians(245), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL - 5))
                .splineToConstantHeading(new Vector2d(-30, -56), Math.toRadians(135), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL - 5))
                .build();

        park[1] = drive.trajectoryBuilder(deposits[3].end(), true)
                .addTemporalMarker(0, () -> lift.grab())
                .addTemporalMarker(.2, () -> lift.trueExtendFourBar())
                .addTemporalMarker(.2, ()-> lift.swivelIn())
                .addTemporalMarker(1.5, () -> lift.retractFourBar())
                .addTemporalMarker(.5, ()-> lift.setSlideTarget(0))
                .lineToLinearHeading(new Pose2d(-30, -36, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL - 5))
                .build();

        park[2] = drive.trajectoryBuilder(deposits[3].end(), true)
                .addTemporalMarker(.8, () -> lift.trueExtendFourBar())
                .addTemporalMarker(0, () -> lift.grab())
                .addTemporalMarker(.8, ()-> lift.swivelIn())
                .addTemporalMarker(2, () -> lift.retractFourBar())
                .addTemporalMarker(.8, ()-> lift.setSlideTarget(0))
                .splineToConstantHeading(new Vector2d(-18, -24), Math.toRadians(90), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL - 5))
                .splineToConstantHeading(new Vector2d(-18, -8), Math.toRadians(-245), SampleMecanumDrive.getVelocityConstraint(22.5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL - 5))
                .splineToSplineHeading(new Pose2d(-30, -8, Math.toRadians(0)), Math.toRadians(-147.5))
                .build();

        while(!isStarted()){
            pos = vision.getPark();
            telemetry.addData("park: ", pos);
            telemetry.update();
        }

        waitForStart();

        liftTime.reset();
        drive.followTrajectoryAsync(preload);

        while (!isStopRequested())
        {
            telemetry.addData("busy :: ", drive.isBusy());
            telemetry.addData("state :: ", auto);
            telemetry.update();
            switch (auto)
            {
                case preload:
                {
                    if (!drive.isBusy() && !goNext)
                    {
                        lift.release();
                        state.reset();
                        goNext = true;
                    }
                    if (state.milliseconds() > 100 && goNext)
                    {
                        drive.followTrajectoryAsync(grabs[cycleNum]);
                        goNext = false;
                        auto = State.grab;
                    }
                    break;
                }
                case deposit:
                {
                    if (!drive.isBusy() && !goNext) {
                        state.reset();
                        goNext = true;
                    }
                    if (state.milliseconds() > 25 && goNext)
                    {
                        lift.release();
                    }
                    if (state.milliseconds() > 250 && goNext) {
                        goNext = false;
                        if (cycleNum < 4)
                            lift.neutralFourBar();
                        state.reset();
                        // DO NOT MOVE THIS STATEMENT UNDER ANY CIRCUMSTANCE OTHERWISE IT WILL RUN TOO MANY TIMES
                        // DO NOT MOVE THIS STATEMENT UNDER ANY CIRCUMSTANCE OTHERWISE IT WILL RUN TOO MANY TIMES
                        cycleNum++;
                        // DO NOT MOVE THIS STATEMENT UNDER ANY CIRCUMSTANCE OTHERWISE IT WILL RUN TOO MANY TIMES
                        // DO NOT MOVE THIS STATEMENT UNDER ANY CIRCUMSTANCE OTHERWISE IT WILL RUN TOO MANY TIMES
                        if (cycleNum < 4)
                        {
                            drive.followTrajectoryAsync(grabs[cycleNum]);
                            auto = State.grab;
                        }
                        else
                        {
                            drive.followTrajectoryAsync(park[pos - 1]);
                            auto = State.park;
                        }
                    }
                    break;
                }
                case grab:
                {
                    if (!drive.isBusy() && !goNext) {
                        state.reset();
                        lift.grab();
                        goNext = true;
                    }
                    if (state.milliseconds() > 300 && goNext)
                    {
                        lift.setSlideTarget(350);
                        lift.neutralFourBar();
                    }
                    if (state.milliseconds() > 400 && goNext) {
                        goNext = false;
                        drive.followTrajectoryAsync(deposits[cycleNum]);
                        auto = State.deposit;
                        state.reset();
                    }
                    break;
                }
                case park:
                {
                    telemetry.addData("afk:", "yeah");
                    telemetry.update();
                    if (!drive.isBusy())
                        stop = liftTime.milliseconds();
                    break;
                }
            }
            drive.update();
            lift.updateLiftLength(liftTime.milliseconds());
            if (!drive.isBusy() && lift.getLiftPos() < 10 && auto == State.park && liftTime.milliseconds() > stop + 750) break;
        }
    }
}