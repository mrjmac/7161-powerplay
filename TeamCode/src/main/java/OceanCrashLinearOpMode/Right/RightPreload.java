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
@Autonomous(name = "RightPreload", group = "Right")
public class RightPreload extends LinearOpMode {

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

    private TrajectorySequence park0;

    enum State {
        preload,
        grabp,
        grab,
        deposit,
        park
    }

    private State auto = State.preload;
    private boolean goNext = false;

    private int pos, cycleNum = 5;
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
                .addTemporalMarker(1.1, () -> lift.setSlideTarget(875))
                .addTemporalMarker(2.1, ()-> lift.swivelStartRight())
                //.splineToSplineHeading(new Pose2d(-40, 35, Math.toRadians(0)), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(60))
                .splineToSplineHeading(new Pose2d(-25.2, -32.3, Math.toRadians(25)), Math.toRadians(15), SampleMecanumDrive.getVelocityConstraint(33, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(30))
                .build();

        grabs[0] = drive.trajectoryBuilder(preload.end())
                .addTemporalMarker(.25, () -> lift.setSlideTarget(120))
                .addTemporalMarker(.5, ()-> lift.trueExtendFourBar())
                .addTemporalMarker(.9, ()-> lift.swivelOut())
                .splineToSplineHeading(new Pose2d(-23.9, -34.3, Math.toRadians(-90)), Math.toRadians(-90), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(-23.9, -47.5, Math.toRadians(-90)), Math.toRadians(-90), SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        deposits[0] = drive.trajectoryBuilder(grabs[0].end())
                .addTemporalMarker(0, () -> lift.extendFourBar())
                .addTemporalMarker(.25, () -> lift.setSlideTarget(850))
                .lineToLinearHeading(new Pose2d(-27.7, -31.1, Math.toRadians(25)), SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(60), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(25))
                .build();

        grabs[1] = drive.trajectoryBuilder(deposits[0].end())
                .addTemporalMarker(.25, () -> lift.setSlideTarget(95))
                .addTemporalMarker(.5, ()-> lift.trueExtendFourBar())
                .addTemporalMarker(.9, ()-> lift.swivelOut())
                .splineToSplineHeading(new Pose2d(-24, -34.3, Math.toRadians(-90)), Math.toRadians(-90), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(-24, -47.1, Math.toRadians(-90)), Math.toRadians(-90), SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        deposits[1] = drive.trajectoryBuilder(grabs[1].end())
                .addTemporalMarker(0, () -> lift.extendFourBar())
                .addTemporalMarker(.25, () -> lift.setSlideTarget(850))
                .lineToLinearHeading(new Pose2d(-27.6, -30.8, Math.toRadians(25)), SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(60), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(25))
                .build();

        grabs[2] = drive.trajectoryBuilder(deposits[1].end())
                .addTemporalMarker(.25, () -> lift.setSlideTarget(50))
                .addTemporalMarker(.5, ()-> lift.trueExtendFourBar())
                .addTemporalMarker(.9, ()-> lift.swivelOut())
                .splineToSplineHeading(new Pose2d(-23.3, -32.3, Math.toRadians(-90)), Math.toRadians(-90), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(-23.3, -48.0, Math.toRadians(-90)), Math.toRadians(-90), SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        deposits[2] = drive.trajectoryBuilder(grabs[2].end())
                .addTemporalMarker(0, () -> lift.extendFourBar())
                .addTemporalMarker(.25, () -> lift.setSlideTarget(850))
                .lineToLinearHeading(new Pose2d(-28.2, -31, Math.toRadians(25)), SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(60), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(25))
                .build();

        grabs[3] = drive.trajectoryBuilder(deposits[2].end())
                .addTemporalMarker(.25, () -> lift.setSlideTarget(30))
                .addTemporalMarker(.5, ()-> lift.trueExtendFourBar())
                .addTemporalMarker(.9, ()-> lift.swivelOut())
                .splineToSplineHeading(new Pose2d(-23.3, -36, Math.toRadians(-90)), Math.toRadians(-90), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(-23.3, -48.0, Math.toRadians(-90)), Math.toRadians(-90), SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        deposits[3] = drive.trajectoryBuilder(grabs[3].end())
                .addTemporalMarker(0, () -> lift.extendFourBar())
                .addTemporalMarker(.25, () -> lift.setSlideTarget(850))
                .lineToLinearHeading(new Pose2d(-28.0, -30.8, Math.toRadians(25)), SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(60), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(25))
                .build();

        park[2] = drive.trajectoryBuilder(preload.end(), true)
                .addTemporalMarker(0, () -> lift.grab())
                .addTemporalMarker(0, ()-> lift.swivelIn())
                .addTemporalMarker(.5, () -> lift.retractFourBar())
                .addTemporalMarker(.5, ()-> lift.setSlideTarget(0))
                .splineToConstantHeading(new Vector2d(-22.5, -45), Math.toRadians(-90), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineTo(new Vector2d(-22.5, -57), Math.toRadians(245), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //.splineToConstantHeading(new Vector2d(-40, -55), Math.toRadians(135), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        park[1] = drive.trajectoryBuilder(preload.end(), true)
                .addTemporalMarker(0, () -> lift.grab())
                .addTemporalMarker(.2, ()-> lift.swivelIn())
                .addTemporalMarker(.9, () -> lift.retractFourBar())
                .addTemporalMarker(.5, ()-> lift.setSlideTarget(0))
                .lineToLinearHeading(new Pose2d(-39, -36, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        park[0] = drive.trajectoryBuilder(preload.end(), true)
                .addTemporalMarker(0, () -> lift.grab())
                .addTemporalMarker(0, ()-> lift.swivelIn())
                .addTemporalMarker(.5, () -> lift.retractFourBar())
                .addTemporalMarker(.8, ()-> lift.setSlideTarget(0))
                .splineToConstantHeading(new Vector2d(-18, -32), Math.toRadians(90), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-18, -14), Math.toRadians(-245), SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(-30, -14, Math.toRadians(0)), Math.toRadians(-147.5))
                .build();

        park0 = drive.trajectorySequenceBuilder(preload.end())
                .addTemporalMarker(0, () -> lift.grab())
                .addTemporalMarker(0, ()-> lift.swivelIn())
                .addTemporalMarker(.5, () -> lift.retractFourBar())
                .addTemporalMarker(.5, ()-> lift.setSlideTarget(0))
                .lineToLinearHeading(new Pose2d(-28.1, -30.1, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(60), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(25))
                .forward(5)
                .strafeLeft(18)
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
                        if (pos == 1)
                        {
                            drive.followTrajectorySequenceAsync(park0);
                        }
                        else
                        {
                            drive.followTrajectoryAsync(park[pos - 1]);
                        }
                        goNext = false;
                        auto = State.park;
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
                            if (pos == 1)
                            {
                                drive.followTrajectorySequenceAsync(park0);
                            }
                            else
                            {
                                drive.followTrajectoryAsync(park[pos - 1]);
                            }
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
                    if (state.milliseconds() > 220 && goNext)
                    {
                        lift.setSlideTarget(250);
                    }
                    if (state.milliseconds() > 350 && goNext) {
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
                    break;
                }
            }
            drive.update();
            lift.updateLiftLength(liftTime.milliseconds());
            if (!drive.isBusy() && lift.getLiftPos() < 100 && auto == State.park) break;
        }
    }
}