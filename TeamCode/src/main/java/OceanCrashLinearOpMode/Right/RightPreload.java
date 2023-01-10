package OceanCrashLinearOpMode.Right;

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
@Autonomous(name = "rightPreload", group = "right")
public class RightPreload extends LinearOpMode {

    private SampleMecanumDrive drive;
    private Drivetrain drivetrain;
    private Lift lift;
    private Vision vision;
    private Intake intake;

    TrajectorySequence fullauto;

    private int pos;
    private double targetPos;
    private double parkPos = -38;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);
        drivetrain = new Drivetrain(this);
        vision = new Vision(this);
        lift = new Lift(this);
        lift.grab();
        intake = new Intake(this);

        Pose2d startingPose = new Pose2d(-72, -36.8, 0);

        drive.setPoseEstimate(startingPose);

        while(!isStarted()){
            pos = vision.getParkRight();
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
                .lineToLinearHeading(new Pose2d(-24.5, -33.3, Math.toRadians(25)), SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> lift.swivelStartLeft())
                .waitSeconds(.5)
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> lift.release())
                .waitSeconds(.25) // 1
                .UNSTABLE_addTemporalMarkerOffset(1.45, () -> targetPos = 60)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> lift.swivelOut())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> lift.trueExtendFourBar())

                //PARK
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> targetPos = 0)
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> lift.swivelIn())
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> lift.grab())
                .UNSTABLE_addTemporalMarkerOffset(.75, () -> lift.retractFourBar())
                .lineToLinearHeading(new Pose2d(-18, -35, Math.toRadians(0)))
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
