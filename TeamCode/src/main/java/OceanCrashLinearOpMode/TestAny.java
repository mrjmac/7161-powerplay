package OceanCrashLinearOpMode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.google.gson.interceptors.JsonPostDeserializer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.genetics.ElitisticListPopulation;

import OceanCrashOpMode.OceanCrashTeleOp;
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

    public static double targetPos = 0;
    public static double cycle = 1;
    public static double cycleTarget = 5;
    public static double parkPos = 0;
    private int turnCount = 1;

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
        turn135
    }

    private boolean first = true;


    ElapsedTime deposit = new ElapsedTime();
    ElapsedTime grab = new ElapsedTime();
    ElapsedTime button = new ElapsedTime();


    private State auto = State.traj1;


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

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startingPose)
                //.addTemporalMarker(.25, () -> targetPos = 700)
                .lineToLinearHeading(new Pose2d(-34, 36, 0))
                .lineToLinearHeading(new Pose2d(-22, 34.541, Math.toRadians(-45)))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end().plus(new Pose2d(0, 0, Math.toRadians(135))))
                //.lineToLinearHeading(new Pose2d(-29.5, 34.5, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-23.6, 54.5, Math.toRadians(90)), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, Math.toRadians(80), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineToLinearHeading(new Pose2d(-22.471, 13.6, Math.toRadians(90)), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, Math.toRadians(80), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .lineToLinearHeading(new Pose2d(-24, 54.5, Math.toRadians(90)), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, Math.toRadians(80), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();






    /*
        TrajectorySequence one = drive.trajectorySequenceBuilder(startingPose)

                .addTemporalMarker(.25, () -> targetPos = 700)
                .lineToLinearHeading(new Pose2d(-36, 36, 0))
                .lineToLinearHeading(new Pose2d(-23.5, 34.5, Math.toRadians(-45)))
                .lineToLinearHeading(new Pose2d(-29.5, 34.5, Math.toRadians(-45)))
                .lineToLinearHeading(new Pose2d(-27, 53.25, Math.toRadians(90)), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, Math.toRadians(80), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-27.5, 34.5, Math.toRadians(-45)))
                .lineToLinearHeading(new Pose2d(-23.5, 34.5, Math.toRadians(-45)))
                .build();

     */

/*
        TrajectorySequence test = drive.trajectorySequenceBuilder(startingPose)
                .addTemporalMarker(.25, () -> lift.setLiftPower(-.4))
                .addTemporalMarker(.75, () -> lift.setLiftPower(0))
                .lineToLinearHeading(new Pose2d(-36, 36, 0))
                /*
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
                //.build();

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
                .lineToLinearHeading(new Pose2d(-24, 36 + parkPos, Math.toRadians(90)))
                .build();

        lift.spinR.setPosition(0.15);
        lift.spinL.setPosition(0.85);
        lift.grab();


        waitForStart();

        drive.followTrajectorySequenceAsync(traj1);

        while (!isStopRequested())
        {
            switch (auto)
            {
                case traj1:
                    if (!drive.isBusy()) {
                        if (gamepad1.dpad_up && button.milliseconds() > 200) {
                            auto = State.deposit;
                            deposit.reset();
                            button.reset();
                        }
                    }
                    break;
                case deposit:
                    lift.release();
                    if (deposit.milliseconds() > 200)
                        if (first) {
                            auto = State.turn135;
                            first = false;
                            drive.turnAsync(Math.toRadians(135));
                        } else {
                            if (cycle != cycleTarget) {
                                if (gamepad1.dpad_up && button.milliseconds() > 200) {
                                    auto = State.turn45;
                                    if (turnCount % 2 == 1)
                                        drive.turnAsync(Math.toRadians(-45));
                                    else
                                        drive.turnAsync(Math.toRadians(45));
                                    cycle++;
                                    button.reset();
                                }
                            } else {
                                if (gamepad1.dpad_up && button.milliseconds() > 200) {
                                    auto = State.park;
                                    drive.followTrajectoryAsync(park);
                                    button.reset();
                                }
                            }

                        }
                    break;
                case turn135:
                    if (!drive.isBusy()) {
                        if (gamepad1.dpad_up && button.milliseconds() > 200) {
                            auto = State.traj2;
                            drive.followTrajectoryAsync(traj2);
                            button.reset();
                        }
                    }
                    break;
                case traj2:
                    if (!drive.isBusy())
                        if (gamepad1.dpad_up && button.milliseconds() > 200) {
                            auto = State.grab;
                            grab.reset();
                            button.reset();
                        }
                    break;
                case grab:
                    if (grab.milliseconds() < 1000)
                        targetPos = 0;
                    if (grab.milliseconds() > 500)
                        lift.grab();
                    if (grab.milliseconds() > 1000)
                        targetPos = 0;
                    if (grab.milliseconds()> 1250) {
                        if (gamepad1.dpad_up && button.milliseconds() > 200) {
                            auto = State.traj3;
                            drive.followTrajectoryAsync(traj3);
                            button.reset();
                        }
                    }
                    break;
                case traj3:
                    if (!drive.isBusy())
                        if (gamepad1.dpad_up && button.milliseconds() > 200) {

                            auto = State.turn45;
                            if (turnCount % 2 == 1)
                                drive.turnAsync(Math.toRadians(-45));
                            else
                                drive.turnAsync(Math.toRadians(45));
                            button.reset();
                        }
                    break;
                case turn45:
                    if (!drive.isBusy())
                        if (gamepad1.dpad_up && button.milliseconds() > 200) {
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
                        if (gamepad1.dpad_up && button.milliseconds() > 200) {
                            auto = State.grab;
                            button.reset();
                        }
                    break;
                case park:
                    if (!drive.isBusy())
                        if (gamepad1.dpad_up && button.milliseconds() > 200) {
                            auto = State.idle;
                            button.reset();
                        }
                    break;
                case idle:
                    telemetry.addData("state:", "FUCK YOU FUCK YOU FUCK YOU FUCK YOU FUCK YOU FUCK YOU FUCK YOU FUCK YOU FUCK YOU FUCK YOU FUCK YOU FUCK YOU FUCK YOU FUCK YOU");
                    telemetry.update();
                    break;
            }
            drive.update();
            lift.setLiftPos(targetPos);
        }
    }
}
