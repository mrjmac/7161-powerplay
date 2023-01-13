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

        Pose2d startingPose = new Pose2d(-72, 37.8, 0);

        drive.setPoseEstimate(startingPose);

        while(!isStarted()){
            pos = vision.getParkLeft();
            telemetry.addData("park: ", pos);
            telemetry.update();
        }

        switch (pos) {
            case 1:
                break;
            case 2:
                parkPos -= 24;
                break;
            case 3:
                parkPos -= 48;
                break;
        }

        fullauto = drive.trajectorySequenceBuilder(startingPose)
                //PRELOAD

                .addTemporalMarker(0, ()-> lift.extendFourBar())
                .addTemporalMarker(0, () -> targetPos = 925)
                .waitSeconds(.25)
                .lineToLinearHeading(new Pose2d(-26, 35.025, Math.toRadians(-25)), SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> lift.swivelStartLeft())
                .waitSeconds(.5)
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> lift.release())
                .waitSeconds(.25) // 1
                .UNSTABLE_addTemporalMarkerOffset(.55, () -> targetPos = 130) // 1.1
                .UNSTABLE_addTemporalMarkerOffset(0, () -> lift.swivelOut())
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> lift.trueExtendFourBar())


                //CYCLE 1
                //.turn(Math.toRadians(115))
                .lineToLinearHeading(new Pose2d(-20.9, 35.3, Math.toRadians(90)), SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-20.9, 47.8, Math.toRadians(90)), SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //.waitSeconds(.3) // 1
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()-> lift.grab())
                .UNSTABLE_addTemporalMarkerOffset(.2, ()-> targetPos = 850)
                .waitSeconds(.3) // 2
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> lift.extendFourBar())
                .lineToLinearHeading(new Pose2d(-26.8, 31.5, Math.toRadians(-25)), SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(.25)
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> lift.release())
                .waitSeconds(.25) // 1
                .UNSTABLE_addTemporalMarkerOffset(.55, () -> targetPos = 110)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> lift.swivelOut())
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> lift.trueExtendFourBar())

                //CYCLE 2
                //.turn(Math.toRadians(115))
                .lineToLinearHeading(new Pose2d(-20.9, 35.3, Math.toRadians(90)), SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-20.9, 46.7, Math.toRadians(90)), SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //.waitSeconds(.3) // 1
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()-> lift.grab())
                .UNSTABLE_addTemporalMarkerOffset(.2, ()-> targetPos = 850)
                .waitSeconds(.3) // 2
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> lift.extendFourBar())
                .lineToLinearHeading(new Pose2d(-25.2, 31.0, Math.toRadians(-25)), SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(.25)
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> lift.release())
                .waitSeconds(.25) // 1
                .UNSTABLE_addTemporalMarkerOffset(.55, () -> targetPos = 90)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> lift.swivelOut())
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> lift.trueExtendFourBar())

                //CYCLE 3
                //.turn(Math.toRadians(115))
                .lineToLinearHeading(new Pose2d(-22.1, 34.3, Math.toRadians(90)), SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-22.1, 45.4, Math.toRadians(90)), SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //.waitSeconds(.3) // 1
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()-> lift.grab())
                .UNSTABLE_addTemporalMarkerOffset(.2, ()-> targetPos = 850)
                .waitSeconds(.3) // 2
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> lift.extendFourBar())
                .lineToLinearHeading(new Pose2d(-26.2, 30.0, Math.toRadians(-25)), SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(.25)
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> lift.release())
                .waitSeconds(.25) // 1
                .UNSTABLE_addTemporalMarkerOffset(.55, () -> targetPos = 70)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> lift.swivelOut())
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> lift.trueExtendFourBar())

                //CYCLE 4
                .lineToLinearHeading(new Pose2d(-20.9, 33.3, Math.toRadians(90)), SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-20.9, 32.0, Math.toRadians(90)), SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(.1, ()-> lift.grab())
                .UNSTABLE_addTemporalMarkerOffset(.4, ()-> targetPos = 300)
                .UNSTABLE_addTemporalMarkerOffset(.5, ()-> lift.swivelIn())
                .UNSTABLE_addTemporalMarkerOffset(2, ()-> lift.startFourBar())
                .waitSeconds(2) // 2
                /*.lineToLinearHeading(new Pose2d(-18, 35, Math.toRadians(0)))
                .waitSeconds(1) // 1

                 */

                //PARK
                /*
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> targetPos = 0)
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> lift.swivelIn())
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> lift.grab())
                .UNSTABLE_addTemporalMarkerOffset(.75, () -> lift.retractFourBar())
                 */
                .lineToConstantHeading(new Vector2d(-20.9, parkPos))
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