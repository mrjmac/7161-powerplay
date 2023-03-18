package OceanCrashLinearOpMode.Left;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import OceanCrashLinearOpMode.Drivetrain;
import OceanCrashLinearOpMode.Intake;
import OceanCrashLinearOpMode.Lift;
import OceanCrashLinearOpMode.Vision;
import OceanCrashRoadrunner.drive.DriveConstants;
import OceanCrashRoadrunner.drive.SampleMecanumDrive;
import OceanCrashRoadrunner.trajectorysequence.TrajectorySequence;

@Config

@Autonomous(name = "LeftMid", group = "Left")
public class LeftMid extends LinearOpMode {

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
    private TrajectorySequence[] grabs = new TrajectorySequence[5];
    private Trajectory[] deposits = new Trajectory[5];

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

        Pose2d startingPose = new Pose2d(-72, 35.3 - (3.0/8), 0);

        drive.setPoseEstimate(startingPose);



        preload = drive.trajectoryBuilder(startingPose)
                .addTemporalMarker(0, ()-> lift.extendFourBar())
                //.addTemporalMarker(.5, () -> lift.setSlideTarget(600))
                .addTemporalMarker(1.5, ()-> lift.swivelStartLeft())
                .splineToSplineHeading(new Pose2d(-47.9, 31.5, Math.toRadians(-25)), Math.toRadians(-15), SampleMecanumDrive.getVelocityConstraint(33, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        grabs[0] = drive.trajectorySequenceBuilder(preload.end())
                //.addTemporalMarker(.4, () -> lift.setSlideTarget(120))
                .addTemporalMarker(.6, ()-> lift.trueExtendFourBar())
                .addTemporalMarker(.9, ()-> lift.swivelOut())
                .splineToConstantHeading(new Vector2d(-47.9, 38.5), Math.toRadians(-15))
                .splineToSplineHeading(new Pose2d(-19, 40, Math.toRadians(90)), Math.toRadians(90), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, Math.toRadians(150), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(-19, 48, Math.toRadians(90)), Math.toRadians(90), SampleMecanumDrive.getVelocityConstraint(15, Math.toRadians(20), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        deposits[0] = drive.trajectoryBuilder(grabs[0].end())
                //.addTemporalMarker(.25, () -> lift.setSlideTarget(600))
                .lineToLinearHeading(new Pose2d(-23.5, 40, Math.toRadians(225)), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, Math.toRadians(140), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        grabs[1] = drive.trajectorySequenceBuilder(deposits[0].end())
                //.addTemporalMarker(.4, () -> lift.setSlideTarget(95))
                .addTemporalMarker(.5, ()-> lift.trueExtendFourBar())
                .addTemporalMarker(.9, ()-> lift.swivelOut())
                //.splineToSplineHeading(new Pose2d(-21, 28, Math.toRadians(180)), Math.toRadians(-180), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-22, 48, Math.toRadians(90)), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, Math.toRadians(140), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-22, 50, Math.toRadians(90)), SampleMecanumDrive.getVelocityConstraint(20, Math.toRadians(30), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        deposits[1] = drive.trajectoryBuilder(grabs[1].end())
                //.addTemporalMarker(.25, () -> lift.setSlideTarget(600))
                .lineToLinearHeading(new Pose2d(-24.5, 42, Math.toRadians(225)), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, Math.toRadians(140), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        grabs[2] = drive.trajectorySequenceBuilder(deposits[1].end())
                //.addTemporalMarker(.4, () -> lift.setSlideTarget(95))
                .addTemporalMarker(.5, ()-> lift.trueExtendFourBar())
                .addTemporalMarker(.9, ()-> lift.swivelOut())
                //.splineToSplineHeading(new Pose2d(-21, 28, Math.toRadians(180)), Math.toRadians(-180), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-25, 50, Math.toRadians(90)), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, Math.toRadians(140), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-25, 54, Math.toRadians(90)), SampleMecanumDrive.getVelocityConstraint(20, Math.toRadians(30), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        deposits[2] = drive.trajectoryBuilder(grabs[2].end())
                //.addTemporalMarker(.25, () -> lift.setSlideTarget(600))
                .lineToLinearHeading(new Pose2d(-25.5, 44, Math.toRadians(225)), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, Math.toRadians(140), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        grabs[3] = drive.trajectorySequenceBuilder(deposits[2].end())
                //.addTemporalMarker(.4, () -> lift.setSlideTarget(95))
                .addTemporalMarker(.5, ()-> lift.trueExtendFourBar())
                .addTemporalMarker(.9, ()-> lift.swivelOut())
                //.splineToSplineHeading(new Pose2d(-21, 28, Math.toRadians(180)), Math.toRadians(-180), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-27, 52, Math.toRadians(90)), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, Math.toRadians(140), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-27, 54, Math.toRadians(90)), SampleMecanumDrive.getVelocityConstraint(20, Math.toRadians(30), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        deposits[3] = drive.trajectoryBuilder(grabs[3].end())
                //.addTemporalMarker(.25, () -> lift.setSlideTarget(600))
                .lineToLinearHeading(new Pose2d(-27.5, 46, Math.toRadians(225)), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, Math.toRadians(140), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        grabs[4] = drive.trajectorySequenceBuilder(deposits[3].end())
                //.addTemporalMarker(.4, () -> lift.setSlideTarget(95))
                .addTemporalMarker(.5, ()-> lift.trueExtendFourBar())
                .addTemporalMarker(.9, ()-> lift.swivelOut())
                //.splineToSplineHeading(new Pose2d(-21, 28, Math.toRadians(180)), Math.toRadians(-180), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-28, 54, Math.toRadians(90)), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, Math.toRadians(140), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-28, 56, Math.toRadians(90)), SampleMecanumDrive.getVelocityConstraint(20, Math.toRadians(30), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        deposits[4] = drive.trajectoryBuilder(grabs[4].end())
                //.addTemporalMarker(.25, () -> lift.setSlideTarget(600))
                .lineToLinearHeading(new Pose2d(-29, 48, Math.toRadians(225)), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, Math.toRadians(140), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        park[0] = drive.trajectoryBuilder(deposits[3].end(), true)
                .addTemporalMarker(0, () -> lift.grab())
                .addTemporalMarker(0, ()-> lift.swivelIn())
                .addTemporalMarker(.5, () -> lift.retractFourBar())
                .addTemporalMarker(.5, ()-> lift.setSlideTarget(0))
                .splineToConstantHeading(new Vector2d(-15, 45), Math.toRadians(90), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineTo(new Vector2d(-15, 55), Math.toRadians(-245), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-30, 55), Math.toRadians(-135), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        park[1] = drive.trajectoryBuilder(deposits[3].end(), true)
                .addTemporalMarker(0, () -> lift.grab())
                .addTemporalMarker(.2, ()-> lift.swivelIn())
                .addTemporalMarker(.9, () -> lift.retractFourBar())
                .addTemporalMarker(.5, ()-> lift.setSlideTarget(0))
                .lineToLinearHeading(new Pose2d(-30, 33, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        park[2] = drive.trajectoryBuilder(deposits[3].end(), true)
                .addTemporalMarker(0, () -> lift.grab())
                .addTemporalMarker(0, ()-> lift.swivelIn())
                .addTemporalMarker(.5, () -> lift.retractFourBar())
                .addTemporalMarker(.8, ()-> lift.setSlideTarget(0))
                .splineToConstantHeading(new Vector2d(-18, 24), Math.toRadians(-90), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-18, 8), Math.toRadians(245), SampleMecanumDrive.getVelocityConstraint(22.5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(-30, 8, Math.toRadians(0)), Math.toRadians(147.5))
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
                        //lift.neutralFourBar();
                        drive.followTrajectorySequenceAsync(grabs[cycleNum]);
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
                        //lift.neutralFourBar();
                        state.reset();
                        // DO NOT MOVE THIS STATEMENT UNDER ANY CIRCUMSTANCE OTHERWISE IT WILL RUN TOO MANY TIMES
                        // DO NOT MOVE THIS STATEMENT UNDER ANY CIRCUMSTANCE OTHERWISE IT WILL RUN TOO MANY TIMES
                        cycleNum++;
                        // DO NOT MOVE THIS STATEMENT UNDER ANY CIRCUMSTANCE OTHERWISE IT WILL RUN TOO MANY TIMES
                        // DO NOT MOVE THIS STATEMENT UNDER ANY CIRCUMSTANCE OTHERWISE IT WILL RUN TOO MANY TIMES
                        if (cycleNum < 5)
                        {
                            drive.followTrajectorySequenceAsync(grabs[cycleNum]);
                            auto = State.grab;
                        }
                        else
                        {
                            //drive.followTrajectoryAsync(park[pos - 1]);
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
                        //lift.setSlideTarget(250);
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