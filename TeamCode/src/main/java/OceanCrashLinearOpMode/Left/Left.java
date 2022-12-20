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

    public static double stall = -.0004;

    private final double turnP45 = .43;
    private final double turnD45 = .40;

    private final double turnP135 = .24;
    private final double turnD135 = .20;

    private final double moveP48 = .442;
    private final double moveP4 = 1;
    private final double moveP20 = .432;


    ElapsedTime deposit = new ElapsedTime();
    ElapsedTime grab = new ElapsedTime();
    ElapsedTime button = new ElapsedTime();

    TrajectorySequence traj1;
    TrajectorySequence park;

    private int pos;
    private double targetPos;
    private double parkPos = 36;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);
        drivetrain = new Drivetrain(this);
        vision = new Vision(this);
        lift = new Lift(this);
        lift.grab();
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
                parkPos += 21;
                break;
            case 2:
                parkPos -= 5;
                break;
            case 3:
                parkPos -= 23;
                break;
        }

        traj1 = drive.trajectorySequenceBuilder(startingPose)
                //PRELOAD

                //.addTemporalMarker(0, ()-> lift.extendFourBar())
                //.addTemporalMarker(0, () -> targetPos = 2700)
                .waitSeconds(.5)
                .lineToLinearHeading(new Pose2d(-26, 37.25, Math.toRadians(-25)))//, SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //.UNSTABLE_addTemporalMarkerOffset(0, ()->lift.swivelStartLeft())
                .waitSeconds(.5)
                //.UNSTABLE_addTemporalMarkerOffset(0, ()-> lift.release())
                .waitSeconds(.5) // 1
                //.UNSTABLE_addTemporalMarkerOffset(.5, () -> targetPos = 350)
                //.UNSTABLE_addTemporalMarkerOffset(.5, () -> lift.swivelOut())


                //CYCLE 1

                .lineToLinearHeading(new Pose2d(-22, 49, Math.toRadians(90)))
                .waitSeconds(.5) // 1
                //.lineToLinearHeading(new Pose2d(-21,49, Math.toRadians(90)), SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))// THIS LINE
                //.lineToLinearHeading(new Pose2d(-20, 46, Math.toRadians(90))) this was commented
                //.UNSTABLE_addTemporalMarkerOffset(.5, ()-> lift.grab())
                //.UNSTABLE_addTemporalMarkerOffset(1.25, ()-> targetPos = 2700)
                .waitSeconds(.5) // 2
                //.lineToLinearHeading(new Pose2d(-20, 42, Math.toRadians(90)))
                //.UNSTABLE_addTemporalMarkerOffset(.2, ()-> targetPos = 2700)
                //.setReversed(true)
                .lineToLinearHeading(new Pose2d(-24, 37.25, Math.toRadians(-25)))//, SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //.setReversed(false)
                //.UNSTABLE_addTemporalMarkerOffset(.2, ()-> lift.swivelStartLeft())
                .waitSeconds(.5)
                //.UNSTABLE_addTemporalMarkerOffset(0, ()-> lift.release())
                .waitSeconds(.5) // 1
                /*.UNSTABLE_addTemporalMarkerOffset(.5, () -> targetPos = 0)
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> lift.swivelIn())
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> lift.grab())
                .UNSTABLE_addTemporalMarkerOffset(.75, () -> lift.retractFourBar())
                 */
                //.lineToLinearHeading(new Pose2d(-20, 36, Math.toRadians(0)))
                //.lineToConstantHeading(new Vector2d(-20, parkPos))


                //CYCLE 2
                .lineToLinearHeading(new Pose2d(-18, 48, Math.toRadians(90)))
                .waitSeconds(.5) //1
                //.lineToLinearHeading(new Pose2d(-19,49, Math.toRadians(90)), SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))// THIS LINE
                //.lineToLinearHeading(new Pose2d(-20, 46, Math.toRadians(90))) this was commented
                //.UNSTABLE_addTemporalMarkerOffset(.5, ()-> lift.grab())
                //.UNSTABLE_addTemporalMarkerOffset(1.25, ()-> targetPos = 2700)
                .waitSeconds(.5) //2
                //.lineToLinearHeading(new Pose2d(-20, 42, Math.toRadians(90)))
                //.UNSTABLE_addTemporalMarkerOffset(.2, ()-> targetPos = 2700)
                //.setReversed(true)
                .lineToLinearHeading(new Pose2d(-21, 36.25, Math.toRadians(-25)))
                //.splineTo(new Vector2d(-21.5, 36.25), Math.toRadians(-205), SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //.setReversed(false)
                //.UNSTABLE_addTemporalMarkerOffset(.2, ()-> lift.swivelStartLeft())
                .waitSeconds(.5)
                //.UNSTABLE_addTemporalMarkerOffset(0, ()-> lift.release())
                .waitSeconds(.5) //1
                /*.UNSTABLE_addTemporalMarkerOffset(.5, () -> targetPos = 0)
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> lift.swivelIn())
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> lift.grab())
                .UNSTABLE_addTemporalMarkerOffset(.75, () -> lift.retractFourBar())
                 */
                //.lineToLinearHeading(new Pose2d(-18, 35, Math.toRadians(0)))
                //.lineToConstantHeading(new Vector2d(-18, parkPos))

                //CYCLE 3
                .lineToLinearHeading(new Pose2d(-15, 47, Math.toRadians(90)))
                .waitSeconds(.5) //1
                //.lineToLinearHeading(new Pose2d(-17,49, Math.toRadians(90)), SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))// THIS LINE
                //.lineToLinearHeading(new Pose2d(-20, 46, Math.toRadians(90))) this was commented
                //.UNSTABLE_addTemporalMarkerOffset(.5, ()-> lift.grab())
                //.UNSTABLE_addTemporalMarkerOffset(1.25, ()-> targetPos = 2700)
                .waitSeconds(.5) //2
                //.lineToLinearHeading(new Pose2d(-20, 42, Math.toRadians(90)))
                //.UNSTABLE_addTemporalMarkerOffset(.2, ()-> targetPos = 2700)
                .lineToLinearHeading(new Pose2d(-18, 35.25, Math.toRadians(-25)))
                //.setReversed(true)
                //.splineTo(new Vector2d(-19.5, 36.25), Math.toRadians(-205), SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //.setReversed(false)
                //.UNSTABLE_addTemporalMarkerOffset(.2, ()-> lift.swivelStartLeft())
                .waitSeconds(.5)
                //.UNSTABLE_addTemporalMarkerOffset(0, ()-> lift.release())
                .waitSeconds(.5) //1
                /*.UNSTABLE_addTemporalMarkerOffset(.5, () -> targetPos = 0)
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> lift.swivelIn())
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> lift.grab())
                .UNSTABLE_addTemporalMarkerOffset(.75, () -> lift.retractFourBar())
                 */
                //.lineToLinearHeading(new Pose2d(-15, 35, Math.toRadians(0)))
                //.lineToConstantHeading(new Vector2d(-15, parkPos))

                //CYCLE 4
                .lineToLinearHeading(new Pose2d(-12, 46, Math.toRadians(90)))
                .waitSeconds(.5) //1
                //.lineToLinearHeading(new Pose2d(-17,49, Math.toRadians(90)), SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))// THIS LINE
                //.lineToLinearHeading(new Pose2d(-20, 46, Math.toRadians(90))) this was commented
                //.UNSTABLE_addTemporalMarkerOffset(.5, ()-> lift.grab())
                //.UNSTABLE_addTemporalMarkerOffset(1.25, ()-> targetPos = 2700)
                .waitSeconds(.5) //2
                //.lineToLinearHeading(new Pose2d(-20, 42, Math.toRadians(90)))
                //.UNSTABLE_addTemporalMarkerOffset(.2, ()-> targetPos = 2700)
                .lineToLinearHeading(new Pose2d(-15, 34.25, Math.toRadians(-25)))
                //.setReversed(true)
                //.splineTo(new Vector2d(-17.5, 36.25), Math.toRadians(-205), SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //.setReversed(false)
                //.UNSTABLE_addTemporalMarkerOffset(.2, ()-> lift.swivelStartLeft())
                .waitSeconds(.5)
                //.UNSTABLE_addTemporalMarkerOffset(0, ()-> lift.release())
                .waitSeconds(.5) //1
                /*.UNSTABLE_addTemporalMarkerOffset(.5, () -> targetPos = 0)
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> lift.swivelIn())
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> lift.grab())
                .UNSTABLE_addTemporalMarkerOffset(.75, () -> lift.retractFourBar())
                 */
                //.lineToLinearHeading(new Pose2d(-16, 35, Math.toRadians(0)))
                //.lineToConstantHeading(new Vector2d(-16, parkPos))

                //CYCLE 5
                .lineToLinearHeading(new Pose2d(-9, 45, Math.toRadians(90)))
                .waitSeconds(.5) //1
                //.lineToLinearHeading(new Pose2d(-17,49, Math.toRadians(90)), SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))// THIS LINE
                //.lineToLinearHeading(new Pose2d(-20, 46, Math.toRadians(90))) this was commented
                //.UNSTABLE_addTemporalMarkerOffset(.5, ()-> lift.grab())
                //.UNSTABLE_addTemporalMarkerOffset(1.25, ()-> targetPos = 2700)
                .waitSeconds(.5) //2
                //.lineToLinearHeading(new Pose2d(-20, 42, Math.toRadians(90)))
                //.UNSTABLE_addTemporalMarkerOffset(.2, ()-> targetPos = 2700)
                .lineToLinearHeading(new Pose2d(-12, 33.25, Math.toRadians(-25)))
                //.setReversed(true)
                //.splineTo(new Vector2d(-17.5, 36.25), Math.toRadians(-205), SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //.setReversed(false)
                //.UNSTABLE_addTemporalMarkerOffset(.2, ()-> lift.swivelStartLeft())
                .waitSeconds(.5)
                //.UNSTABLE_addTemporalMarkerOffset(0, ()-> lift.release())
                .waitSeconds(.5) //1
                /*.UNSTABLE_addTemporalMarkerOffset(.5, () -> targetPos = 0)
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> lift.swivelIn())
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> lift.grab())
                .UNSTABLE_addTemporalMarkerOffset(.75, () -> lift.retractFourBar())
                 */

                //PARK
                .lineToLinearHeading(new Pose2d(-16, 35, Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(-16, parkPos))

                .build();


/*
        park = drive.trajectorySequenceBuilder(traj1.end())
                .lineToConstantHeading(new Vector2d(-24, 34 + parkPos))
                .build();
*/


        drive.followTrajectorySequenceAsync(traj1);
        waitForStart();

        while (!isStopRequested())
        {
            drive.update();
            lift.setLiftPos(targetPos);
            if (!drive.isBusy() && lift.getLiftPos() < 100) break;
        }
    }
}
