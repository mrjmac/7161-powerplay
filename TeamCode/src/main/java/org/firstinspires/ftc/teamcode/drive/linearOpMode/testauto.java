package org.firstinspires.ftc.teamcode.drive.linearOpMode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import OceanCrashLinearOpMode.Intake;
import OceanCrashLinearOpMode.Lift;
import OceanCrashLinearOpMode.Vision;

@Autonomous(name = "dadbot movement")
public class testauto extends LinearOpMode {

    private SampleMecanumDrive drivetrain;
    private Lift lift;
    private Vision vision;
    private Intake intake;
    int pos;

    @Override
    public void runOpMode() throws InterruptedException {

        drivetrain = new SampleMecanumDrive(hardwareMap);
        lift = new Lift(this);
        vision = new Vision(this);
        intake = new Intake(this);

        Pose2d startingPose = new Pose2d(0, 0, 0);

        drivetrain.setPoseEstimate(startingPose);

        TrajectorySequence test = drivetrain.trajectorySequenceBuilder(startingPose)
                .lineToLinearHeading(new Pose2d(48, 48, Math.toRadians(90)))

                .addTemporalMarker(() -> intake.startIntake(1))

                .waitSeconds(3)

                .addTemporalMarker(() -> intake.startIntake(0))

                .turn(Math.toRadians(180))
                .waitSeconds(2)

                .lineToLinearHeading(new Pose2d(0, 0, 0))



                .build();

        //drivetrain.setPoseEstimate(startingPose);

        waitForStart();

        if (!isStopRequested())
        {
            //drivetrain.setPoseEstimate(startingPose);
            drivetrain.followTrajectorySequence(test);
        }



    }



}
