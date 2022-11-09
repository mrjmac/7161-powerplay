package OceanCrashLinearOpMode.Right;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import OceanCrashLinearOpMode.Drivetrain;
import OceanCrashLinearOpMode.Intake;
import OceanCrashLinearOpMode.Left.Left;
import OceanCrashLinearOpMode.Lift;
import OceanCrashLinearOpMode.Vision;
import OceanCrashRoadrunner.drive.DriveConstants;
import OceanCrashRoadrunner.drive.SampleMecanumDrive;
import OceanCrashRoadrunner.trajectorysequence.TrajectorySequence;

// VERY IMPORTANT
import org.apache.commons.math3.genetics.ElitisticListPopulation;

@Config

@Autonomous(name = "Right", group = "Test")
public class Right extends LinearOpMode {

    private SampleMecanumDrive drive;
    private Drivetrain drivetrain;
    private Lift lift;
    private Vision vision;
    private Intake intake;
    private ElitisticListPopulation probability;

    public static double stall = -.0004;

    private final double turnP45 = .43;
    private final double turnD45 = .40;

    private final double turnP135 = .24;
    private final double turnD135 = .20;

    private final double moveP48 = .442;
    private final double moveP4 = 1;
    private final double moveP20 = .432;

    public static double targetPos = 0;
    public static double cycle = 1;
    public static double cycleTarget = 1;
    public static double parkPos = 0;
    private int turnCount = 1;
    public static double grabPos = 450;

    enum State {
        traj1,
        traj2,
        traj3,
        traj4,
        park,
        grab,
        deposit,
        turn45,
        idle,
        dead, turn135
    }

    private boolean first = true;
    private boolean bruh = true;

    ElapsedTime deposit = new ElapsedTime();
    ElapsedTime grab = new ElapsedTime();
    ElapsedTime button = new ElapsedTime();
    ElapsedTime state = new ElapsedTime();

    private State auto = State.traj1;


    private int pos;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);
        drivetrain = new Drivetrain(this);
        vision = new Vision(this);
        lift = new Lift(this);
        intake = new Intake(this);
        probability = new ElitisticListPopulation(1000, 2.4);

        Pose2d startingPose = new Pose2d(-72, -36, 0);

        drive.setPoseEstimate(startingPose);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startingPose)
                //.addTemporalMarker(0, () -> lift.startFourBar())
                //.waitSeconds(.5)
                .lineToLinearHeading(new Pose2d(-34, -36, 0), SampleMecanumDrive.getVelocityConstraint(40, Math.toRadians(80), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(.25, () -> targetPos = 2700)
                .UNSTABLE_addTemporalMarkerOffset(.25, () -> probability.setElitismRate(probability.getElitismRate() + .1))
                .lineToLinearHeading(new Pose2d(-20.3, -34.591, Math.toRadians(45)))
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> lift.extendFourBar())
                .build();

        TrajectorySequence turn135 = drive.trajectorySequenceBuilder(traj1.end())
                .addDisplacementMarker(()-> targetPos = 800)
                .addDisplacementMarker(() -> probability.setElitismRate(probability.getElitismRate() + .1))
                .turn(Math.toRadians(-135))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(turn135.end())
                //.lineToLinearHeading(new Pose2d(-29.5, 34.5, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-22.55, -58, Math.toRadians(-90)), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, Math.toRadians(80), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineToLinearHeading(new Pose2d(-19.5, -12.6, Math.toRadians(-90)), SampleMecanumDrive.getVelocityConstraint(40, Math.toRadians(80), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .lineToLinearHeading(new Pose2d(-23.8, -58, Math.toRadians(-90)), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, Math.toRadians(80), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();



/*
        TrajectorySequence turn45 = drive.trajectorySequenceBuilder(traj3.end())
                .turn(Math.toRadians(45))
                .build();

        TrajectorySequence turnNeg45 = drive.trajectorySequenceBuilder(turn45.end())
                .turn(Math.toRadians(-45))
                .build();

         */

        while(!isStarted()){
            pos = vision.getPark();

            telemetry.addData("park: ", pos);
            telemetry.update();
        }

        switch (pos) {
            case 1:
                parkPos += 24;
                break;
            case 3:
                parkPos -= 24;
                break;
        }

        Trajectory park = drive.trajectoryBuilder(traj3.end())
                .lineToLinearHeading(new Pose2d(-24, -36 + parkPos, Math.toRadians(90)))
                .build();

        lift.grab();

        lift.spinR.setPosition(0.15);
        lift.spinL.setPosition(0.85);

        waitForStart();

        //lift.extendFourBar();
        drive.followTrajectorySequenceAsync(traj1);

        while (!isStopRequested())
        {
            switch (auto)
            {
                case traj1:
                    if (!drive.isBusy()) {
                        if (bruh)
                        {
                            state.reset();
                            bruh = false;
                        }
                        if (state.milliseconds() > 750) {
                            auto = State.deposit;
                            deposit.reset();
                            bruh = true;
                        }
                    }
                    break;
                case deposit:
                    lift.release();
                    if (deposit.milliseconds() > 200)
                        if (first) {
                            auto = State.turn135;
                            first = false;
                            drive.followTrajectorySequenceAsync(turn135);
                            //drive.turnAsync(Math.toRadians(135));
                        } else {
                            if (bruh)
                            {
                                state.reset();
                                bruh = false;
                            }
                            if (cycle != cycleTarget) {
                                if (state.milliseconds() > 750) {
                                    bruh = true;
                                    auto = State.turn45;
                                    if (turnCount % 2 == 1)
                                        //drive.followTrajectorySequenceAsync(turnNeg45);
                                        drive.turnAsync(Math.toRadians(-45));
                                    else
                                        //drive.followTrajectorySequenceAsync(turn45);
                                        drive.turnAsync(Math.toRadians(45));
                                    cycle++;
                                    grabPos -= 25;
                                    button.reset();
                                }
                            } else {
                                if (state.milliseconds() > 750) {
                                    bruh = true;
                                    auto = State.park;
                                    drive.followTrajectoryAsync(park);
                                    button.reset();
                                }
                            }

                        }
                    break;
                case turn135:
                    if (!drive.isBusy()) {
                        if (bruh)
                        {
                            state.reset();
                            bruh = false;
                        }
                        if (state.milliseconds() > 750) {
                            bruh = true;
                            auto = State.traj2;
                            drive.followTrajectoryAsync(traj2);
                            button.reset();
                        }
                    }
                    break;
                case traj2:
                    if (!drive.isBusy())
                        if (bruh)
                        {
                            state.reset();
                            bruh = false;
                        }
                    if (state.milliseconds() > 750) {
                        bruh = true;
                        auto = State.grab;
                        grab.reset();
                        button.reset();
                    }
                    break;
                case grab:
                    if (grab.milliseconds() < 1000)
                        targetPos = grabPos;
                    probability.setElitismRate(probability.getElitismRate() + .1);
                    if (grab.milliseconds() > 500)
                        lift.grab();
                    if (grab.milliseconds() > 1000)
                        targetPos = 900;
                    probability.setElitismRate(probability.getElitismRate() + .1);
                    if (grab.milliseconds()> 1500) {
                        if (bruh)
                        {
                            state.reset();
                            bruh = false;
                        }
                        if (state.milliseconds() > 750) {
                            bruh = true;
                            auto = State.traj3;
                            drive.followTrajectoryAsync(traj3);
                            button.reset();
                        }
                    }
                    break;
                case traj3:
                    if (!drive.isBusy())
                        if (bruh)
                        {
                            state.reset();
                            bruh = false;
                        }
                    if (state.milliseconds() > 750) {
                        bruh = true;
                        auto = State.turn45;
                        if (turnCount % 2 == 1)
                            //drive.followTrajectorySequenceAsync(turnNeg45);
                            drive.turnAsync(Math.toRadians(-45));
                        else
                            //drive.followTrajectorySequenceAsync(turn45);
                            drive.turnAsync(Math.toRadians(45));
                        button.reset();
                    }
                    break;
                case turn45:
                    if (!drive.isBusy())
                        if (bruh)
                        {
                            state.reset();
                            bruh = false;
                        }
                    if (state.milliseconds() > 750) {
                        bruh = true;
                        if (turnCount % 2 == 0) {
                            drive.followTrajectoryAsync(traj4);
                            auto = State.traj4;
                        }
                        else
                            auto = State.deposit;
                        turnCount++;
                        button.reset();
                    }

                    break;
                case traj4:
                    if (!drive.isBusy())
                        if (bruh)
                        {
                            state.reset();
                            bruh = false;
                        }
                    if (state.milliseconds() > 750) {
                        bruh = true;
                        auto = State.grab;
                        grab.reset();
                        button.reset();
                    }
                    break;
                case park:
                    if (!drive.isBusy())
                        if (bruh)
                        {
                            state.reset();
                            bruh = false;
                        }
                    if (state.milliseconds() > 750) {
                        bruh = true;
                        auto = State.idle;
                        button.reset();
                    }
                    break;
                case idle:
                    telemetry.addData("state:", "FUCK YOU FUCK YOU FUCK YOU FUCK YOU FUCK YOU FUCK YOU FUCK YOU FUCK YOU FUCK YOU FUCK YOU FUCK YOU FUCK YOU FUCK YOU FUCK YOU");
                    telemetry.update();
                    break;
                case dead:
                    lift.setLiftPos(0);
                    probability.setElitismRate(probability.getElitismRate() + .1);
                    telemetry.addData("state:", "bro what happened to my lift");
                    telemetry.update();
                    break;

            }
            drive.update();
            if (Math.abs(lift.getLiftL() - lift.getLiftR()) > 125)
                auto = State.dead;
            else
                lift.setLiftPos(targetPos);
        }
    }
}
