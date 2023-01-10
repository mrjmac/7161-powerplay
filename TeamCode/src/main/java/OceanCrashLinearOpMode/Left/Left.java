package OceanCrashLinearOpMode.Left;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
@Autonomous(name = "Left", group = "Left")
public class Left extends LinearOpMode {

    private SampleMecanumDrive drive;
    private Drivetrain drivetrain;
    private Lift lift;
    private Vision vision;
    private Intake intake;

    TrajectorySequence fullauto;

    private int pos;
    private double targetPos;
    private double parkPos = 38;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);
        drivetrain = new Drivetrain(this);
        vision = new Vision(this);
        lift = new Lift(this);
        lift.grab();
        intake = new Intake(this);

        Pose2d startingPose = new Pose2d(-72, 36.8, 0);

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
            case 2:
                parkPos -= 5;
                break;
            case 3:
                parkPos -= 23;
                break;
        }

        fullauto = drive.trajectorySequenceBuilder(startingPose)
                //PRELOAD
                .addTemporalMarker(0, ()-> lift.extendFourBar())
                .addTemporalMarker(0, () -> targetPos = 925)
                .waitSeconds(.25)
                .lineToLinearHeading(new Pose2d(-24.5, 33.3, Math.toRadians(-25)), SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> lift.swivelStartLeft())
                .waitSeconds(.5)
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> lift.release())
                .waitSeconds(.25) // 1
                .UNSTABLE_addTemporalMarkerOffset(1.45, () -> targetPos = 70)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> lift.swivelOut())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> lift.trueExtendFourBar())


                //CYCLE 1
                .lineToLinearHeading(new Pose2d(-20, 47.5, Math.toRadians(90)), SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(.3) // 1
                .UNSTABLE_addTemporalMarkerOffset(.5, ()-> lift.grab())
                .UNSTABLE_addTemporalMarkerOffset(.8, ()-> targetPos = 925)
                .waitSeconds(1) // 2
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> lift.extendFourBar())
                .lineToLinearHeading(new Pose2d(-25.3, 32.15, Math.toRadians(-25)), SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(.5)
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> lift.release())
                .waitSeconds(.3) // 1
                .UNSTABLE_addTemporalMarkerOffset(1.45, () -> targetPos = 60)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> lift.swivelOut())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> lift.trueExtendFourBar())

                //CYCLE 2
                .lineToLinearHeading(new Pose2d(-18.1, 47.3, Math.toRadians(90)), SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(.3) // 1
                .UNSTABLE_addTemporalMarkerOffset(.5, ()-> lift.grab())
                .UNSTABLE_addTemporalMarkerOffset(.8, ()-> targetPos = 925)
                .waitSeconds(1) // 2
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> lift.extendFourBar())
                .lineToLinearHeading(new Pose2d(-23, 31.5, Math.toRadians(-25)), SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(.5)
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> lift.release())
                .waitSeconds(.3) // 1
                .UNSTABLE_addTemporalMarkerOffset(1.45, () -> targetPos = 40)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> lift.swivelOut())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> lift.trueExtendFourBar())

                //CYCLE 3
                .lineToLinearHeading(new Pose2d(-16.2, 47.1, Math.toRadians(90)), SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(.3) // 1
                .UNSTABLE_addTemporalMarkerOffset(.5, ()-> lift.grab())
                .UNSTABLE_addTemporalMarkerOffset(.8, ()-> targetPos = 925)
                .waitSeconds(1) // 2
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> lift.extendFourBar())
                .lineToLinearHeading(new Pose2d(-20.7, 30.85, Math.toRadians(-25)), SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(.5)
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> lift.release())
                .waitSeconds(.5) // 1
                .UNSTABLE_addTemporalMarkerOffset(1.45, () -> targetPos = 30)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> lift.swivelOut())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> lift.lastCycleFourBar())

                //CYCLE 4
                .lineToLinearHeading(new Pose2d(-13.8, 46.7, Math.toRadians(90)), SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(.3) // 1
                .UNSTABLE_addTemporalMarkerOffset(.5, ()-> lift.grab())
                .UNSTABLE_addTemporalMarkerOffset(.8, ()-> targetPos = 925)
                .waitSeconds(1) // 2
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> lift.extendFourBar())
                .lineToLinearHeading(new Pose2d(-18.4, 30.3, Math.toRadians(-25)), SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(.5)
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> lift.release())
                .waitSeconds(.1) // 1

                //PARK
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> targetPos = 0)
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> lift.swivelIn())
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> lift.grab())
                .UNSTABLE_addTemporalMarkerOffset(.75, () -> lift.retractFourBar())
                .lineToLinearHeading(new Pose2d(-18, 35, Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(-17, parkPos))

                .build();

        drive.followTrajectorySequenceAsync(fullauto);
        waitForStart();

        while (!isStopRequested())
        {
            drive.update();
            lift.setLiftPos(targetPos);
            if (!drive.isBusy() && lift.getLiftPos() < 100) break;
        }
    }
}
