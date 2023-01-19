package OceanCrashLinearOpMode.Left;

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
@Autonomous(name = "LeftEditedSpeed", group = "Left")
public class LeftEditedSpeed extends LinearOpMode {

    private SampleMecanumDrive drive;
    private Drivetrain drivetrain;
    private Lift lift;
    private Vision vision;
    private Intake intake;

    private Trajectory preload, firstGrab, deposit, grab, park, firstDeposit;

    private Trajectory[] grabs = new Trajectory[5];
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

        Pose2d startingPose = new Pose2d(-72, 35.3, 0);

        drive.setPoseEstimate(startingPose);

        while(!isStarted()){
            pos = vision.getParkLeft();
            telemetry.addData("park: ", pos);
            telemetry.update();
        }

        switch (pos) {
            case 1:
                parkPos += 15.5;
                break;
            case 2:
                parkPos -= 6;
                break;
            case 3:
                parkPos -= 29;
                break;
        }

        preload = drive.trajectoryBuilder(startingPose)
                .addTemporalMarker(0, ()-> lift.extendFourBar())
                .addTemporalMarker(0, () -> lift.setSlideTarget(850))
                .addTemporalMarker(2.1, ()-> lift.swivelStartLeft())
                //.addTemporalMarker(2.4,()->lift.release())
                //.splineToSplineHeading(new Pose2d(-44.3, 34.3, Math.toRadians(0)), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(30, 35, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(-24.8, 33.6, Math.toRadians(-25)), Math.toRadians(-15), SampleMecanumDrive.getVelocityConstraint(28, 35, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(30))
                //.addDisplacementMarker(() -> drive.followTrajectoryAsync(firstGrab))
                .build();

        grabs[0] = drive.trajectoryBuilder(preload.end())
                .addTemporalMarker(.25, () -> lift.setSlideTarget(90))
                .addTemporalMarker(.5, ()-> lift.trueExtendFourBar())
                .addTemporalMarker(.9, ()-> lift.swivelOut())
                .splineToSplineHeading(new Pose2d(-21.1, 34.3, Math.toRadians(90)), Math.toRadians(90), SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(-21.1, 47.2, Math.toRadians(90)), Math.toRadians(90),SampleMecanumDrive.getVelocityConstraint(15, Math.toRadians(10), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        deposits[0] = drive.trajectoryBuilder(grabs[0].end())
                //.addTemporalMarker(0, () -> lift.setSlideTarget(875))
                .addTemporalMarker(1, () -> lift.extendFourBar())
                .lineToLinearHeading(new Pose2d(-28.3, 33.1, Math.toRadians(-25)), SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(70), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        grabs[1] = drive.trajectoryBuilder(deposits[0].end())
                .addTemporalMarker(0.25, () -> lift.setSlideTarget(80))
                .addTemporalMarker(.5, ()-> lift.trueExtendFourBar())
                .addTemporalMarker(.9, ()-> lift.swivelOut())
                .splineToSplineHeading(new Pose2d(-21.1, 34.3, Math.toRadians(90)), Math.toRadians(90), SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(-21.1, 47.2, Math.toRadians(90)), Math.toRadians(90),SampleMecanumDrive.getVelocityConstraint(15, Math.toRadians(10), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        deposits[1] = drive.trajectoryBuilder(grabs[1].end())
               //.addTemporalMarker(0, () -> lift.setSlideTarget(875))
                .addTemporalMarker(1, () -> lift.extendFourBar())
                .lineToLinearHeading(new Pose2d(-27.3, 30.7, Math.toRadians(-25)), SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(70), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        grabs[2] = drive.trajectoryBuilder(deposits[1].end())
                .addTemporalMarker(0.25, () -> lift.setSlideTarget(70))
                .addTemporalMarker(.5, ()-> lift.trueExtendFourBar())
                .addTemporalMarker(.9, ()-> lift.swivelOut())
                .splineToSplineHeading(new Pose2d(-19.6, 32.3, Math.toRadians(90)), Math.toRadians(90), SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(-19.6, 45.2, Math.toRadians(90)), Math.toRadians(90),SampleMecanumDrive.getVelocityConstraint(15, Math.toRadians(10), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        deposits[2] = drive.trajectoryBuilder(grabs[2].end())
                //.addTemporalMarker(0, () -> lift.setSlideTarget(875))
                .addTemporalMarker(1, () -> lift.extendFourBar())
                .lineToLinearHeading(new Pose2d(-25.3, 29.9, Math.toRadians(-25)), SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(70), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        grabs[3] = drive.trajectoryBuilder(deposits[2].end())
                .addTemporalMarker(0.25, () -> lift.setSlideTarget(60))
                .addTemporalMarker(.5, ()-> lift.trueExtendFourBar())
                .addTemporalMarker(.9, ()-> lift.swivelOut())
                .splineToSplineHeading(new Pose2d(-18.3, 30, Math.toRadians(90)), Math.toRadians(90), SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(-18.3, 43.6, Math.toRadians(90)), Math.toRadians(90),SampleMecanumDrive.getVelocityConstraint(15, Math.toRadians(10), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        deposits[3] = drive.trajectoryBuilder(grabs[3].end())
                //.addTemporalMarker(0, () -> lift.setSlideTarget(875))
                .addTemporalMarker(1, () -> lift.extendFourBar())
                .lineToLinearHeading(new Pose2d(-24, 28.5, Math.toRadians(-25)), SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(70), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        park = drive.trajectoryBuilder(deposits[3].end())
                .addTemporalMarker(0, ()-> lift.setSlideTarget(0))
                .lineToLinearHeading(new Pose2d(-24, 29.5, Math.toRadians(-25)), SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(70), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

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
                        lift.release();
                        goNext = true;
                    }
                    if (state.milliseconds() > 200 && goNext) {
                        goNext = false;
                        auto = State.grab;
                        state.reset();
                        cycleNum++;
                        if (cycleNum < 4)
                        {
                            drive.followTrajectoryAsync(grabs[cycleNum]);
                        }
                        else
                        {
                            drive.followTrajectoryAsync(park);
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
                    if (state.milliseconds() > 10 && goNext)
                    {
                        lift.setSlideTarget(875);
                    }
                    if (state.milliseconds() > 200 && goNext) {
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