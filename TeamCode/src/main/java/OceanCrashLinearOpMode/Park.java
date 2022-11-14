package OceanCrashLinearOpMode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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

// VERY IMPORTANT, BUT NOT FOR THIS AUTO
import org.apache.commons.math3.genetics.ElitisticListPopulation;

@Config

@Autonomous(name = "Park", group = "Test")
public class Park extends LinearOpMode {

    private SampleMecanumDrive drive;
    private Drivetrain drivetrain;
    private Lift lift;
    private Vision vision;
    private Intake intake;

    private Boolean running = true;

    public static double stall = -.0004;

    public static double parkPos = 0;

    enum State {
        traj1,
        idle
    }

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
        if (parkPos == 0)
            parkPos = 1;

        TrajectorySequence traj1_2 = drive.trajectorySequenceBuilder(startingPose)
                .lineToLinearHeading(new Pose2d(-46, 36, 0), SampleMecanumDrive.getVelocityConstraint(20, Math.toRadians(80), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(30))
                //.lineToLinearHeading(new Pose2d(-44, 36, Math.toRadians(90)))
                .build();

        TrajectorySequence traj1_1 = drive.trajectorySequenceBuilder(startingPose)
                .lineToLinearHeading(new Pose2d(-46, 36, 0), SampleMecanumDrive.getVelocityConstraint(20, Math.toRadians(80), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(30))
                //.lineToLinearHeading(new Pose2d(-44, 60, Math.toRadians(90)), SampleMecanumDrive.getVelocityConstraint(20, Math.toRadians(80), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(30))
                .turn(Math.toRadians(90))
                .forward(24)
                .build();

        TrajectorySequence traj1_3 = drive.trajectorySequenceBuilder(startingPose)
                .lineToLinearHeading(new Pose2d(-46, 36, 0), SampleMecanumDrive.getVelocityConstraint(20, Math.toRadians(80), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(30))
                //.lineToLinearHeading(new Pose2d(-44, 12, Math.toRadians(90)), SampleMecanumDrive.getVelocityConstraint(20, Math.toRadians(80), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(30))
                .turn(Math.toRadians(-90))
                .forward(24)
                .build();

        lift.grab();

        //lift.spinR.setPosition(0.15);
        //lift.spinL.setPosition(0.85);

        waitForStart();

        //lift.extendFourBar();
        if (pos == 1)
            drive.followTrajectorySequenceAsync(traj1_1);
        else if (pos == 2)
            drive.followTrajectorySequenceAsync(traj1_2);
        else
            drive.followTrajectorySequenceAsync(traj1_3);


        while (!isStopRequested() && running)
        {
            switch (auto)
            {
                case traj1:
                    if (!drive.isBusy())
                        auto = State.idle;
                    break;
                case idle:
                    telemetry.addData("state:", "im done");
                    telemetry.update();
                    running = false;
                    break;
            }
            drive.update();
        }
    }
}
