package OceanCrashLinearOpMode.Left;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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


    ElapsedTime liftTime = new ElapsedTime();

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
                parkPos += 15.5;
                break;
            case 2:
                parkPos -= 6;
                break;
            case 3:
                parkPos -= 29;
                break;
        }

        fullauto = drive.trajectorySequenceBuilder(startingPose)
                //PRELOAD
                .addTemporalMarker(0, ()->liftTime.reset())
                .addTemporalMarker(0, ()-> lift.extendFourBar())
                .addTemporalMarker(0, () -> lift.setSlideTarget(950))
                .waitSeconds(.25)
                .lineToLinearHeading(new Pose2d(-26.8, 32.3, Math.toRadians(-25)), SampleMecanumDrive.getVelocityConstraint(37, 50, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> lift.swivelStartLeft())
                .waitSeconds(.25)
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> lift.release())
                .waitSeconds(.25) // 1
                .UNSTABLE_addTemporalMarkerOffset(.75, () -> lift.setSlideTarget(135)) // 1.1
                .UNSTABLE_addTemporalMarkerOffset(0, () -> lift.swivelOut())
                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> lift.trueExtendFourBar())


                //CYCLE 1
                //.turn(Math.toRadians(115))
                .lineToLinearHeading(new Pose2d(-21.1, 34.3, Math.toRadians(90)), SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-21.1, 48.6, Math.toRadians(90)), SampleMecanumDrive.getVelocityConstraint(15, Math.toRadians(10), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //.waitSeconds(.3) // 1
                .UNSTABLE_addTemporalMarkerOffset(0.05, ()-> lift.grab())
                .UNSTABLE_addTemporalMarkerOffset(.35, ()-> lift.setSlideTarget(950))
                .waitSeconds(.45) // 2
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> lift.extendFourBar())
                .lineToLinearHeading(new Pose2d(-25.9, 31.1, Math.toRadians(-25)), SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(70), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(.25)
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> lift.release())
                .waitSeconds(.25) // 1
                .UNSTABLE_addTemporalMarkerOffset(.55, () -> lift.setSlideTarget(120))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> lift.swivelOut())
                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> lift.trueExtendFourBar())

                //CYCLE 2
                //.turn(Math.toRadians(115))
                .lineToLinearHeading(new Pose2d(-21.1, 33.3, Math.toRadians(90)), SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-21.1, 48.6, Math.toRadians(90)), SampleMecanumDrive.getVelocityConstraint(15, Math.toRadians(10), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //.waitSeconds(.3) // 1
                .UNSTABLE_addTemporalMarkerOffset(0.05, ()-> lift.grab())
                .UNSTABLE_addTemporalMarkerOffset(.35, ()-> lift.setSlideTarget(950))
                .waitSeconds(.45) // 2
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> lift.extendFourBar())
                .lineToLinearHeading(new Pose2d(-25.3, 30.7, Math.toRadians(-25)), SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(70), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(.25)
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> lift.release())
                .waitSeconds(.25) // 1
                .UNSTABLE_addTemporalMarkerOffset(.55, () -> lift.setSlideTarget(105))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> lift.swivelOut())
                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> lift.trueExtendFourBar())

                //CYCLE 3
                //.turn(Math.toRadians(115))
                .lineToLinearHeading(new Pose2d(-19.6, 32.3, Math.toRadians(90)), SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-19.6, 47.6, Math.toRadians(90)), SampleMecanumDrive.getVelocityConstraint(15, Math.toRadians(10), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //.waitSeconds(.3) // 1
                .UNSTABLE_addTemporalMarkerOffset(0.05, ()-> lift.grab())
                .UNSTABLE_addTemporalMarkerOffset(.22, ()-> lift.setSlideTarget(950))
                .waitSeconds(.3) // 2
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> lift.extendFourBar())
                .lineToLinearHeading(new Pose2d(-22.8, 27.9, Math.toRadians(-25)), SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(70), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(.25)
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> lift.release())
                .waitSeconds(.25) // 1
                .UNSTABLE_addTemporalMarkerOffset(.55, () -> lift.setSlideTarget(90))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> lift.swivelOut())
                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> lift.trueExtendFourBar())

                /*
                .UNSTABLE_addTemporalMarkerOffset(.25, ()-> targetPos = 150)
                .UNSTABLE_addTemporalMarkerOffset(.25, ()-> lift.swivelFourBar())
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> lift.swivelIn())
                .UNSTABLE_addTemporalMarkerOffset(.8, ()-> lift.grabFourBar())
                .UNSTABLE_addTemporalMarkerOffset(0.4, ()-> targetPos = 80)
                .UNSTABLE_addTemporalMarkerOffset(1.2, ()-> lift.retractFourBar())
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> lift.release())
                .UNSTABLE_addTemporalMarkerOffset(2, ()-> lift.grabFourBar())
                 */



                //CYCLE 4

                /*
                .lineToLinearHeading(new Pose2d(-18.3, 20, Math.toRadians(90)), SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-18.3, 45.6, Math.toRadians(90)), SampleMecanumDrive.getVelocityConstraint(15, Math.toRadians(10), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0.05, ()-> lift.grab())
                .UNSTABLE_addTemporalMarkerOffset(.25, ()-> targetPos = 150)
                .UNSTABLE_addTemporalMarkerOffset(.2, ()-> lift.swivelFourBar())
                .waitSeconds(1)
                 */
                //.UNSTABLE_addTemporalMarkerOffset(0.4, ()-> targetPos = 60)
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> lift.swivelIn())
                .UNSTABLE_addTemporalMarkerOffset(2, ()-> lift.retractFourBar())
                .lineToLinearHeading(new Pose2d(-20.9, 38, Math.toRadians(-90)))
                .waitSeconds(.5)
                .lineToConstantHeading(new Vector2d(-20.9, parkPos))

                //.waitSeconds(.1) // 2
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
                //.lineToConstantHeading(new Vector2d(-20.9, parkPos))
                .build();

        drive.followTrajectorySequenceAsync(fullauto);
        waitForStart();

        while (!isStopRequested())
        {
            drive.update();
            lift.setSlideTarget(liftTime.milliseconds());
            if (!drive.isBusy() && lift.getLiftPos() < 100) break;
        }
    }
}