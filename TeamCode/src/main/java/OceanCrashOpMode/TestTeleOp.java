package OceanCrashOpMode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.util.Arrays;
import java.util.List;

import OceanCrashLinearOpMode.Right.Right;
import OceanCrashRoadrunner.drive.SampleMecanumDrive;


@Config
@TeleOp(name = "TestTele", group = "opMode")
public class TestTeleOp extends OceanCrashOpMode{


    private boolean angleLock = false;
    private final ElapsedTime angle = new ElapsedTime();
    private double lockedAngle;

    public void loop() {
        if (gamepad1.a && !angleLock && angle.milliseconds() > 200) {
            angleLock = true;
            lockedAngle = getGyroYaw();
            angle.reset();
        }
        else if (gamepad1.a && angleLock && angle.milliseconds() > 200) {
            angleLock = false;
            angle.reset();
        }

        // DRIVE
        if (Math.abs(gamepad1.left_stick_x) > .1 || Math.abs(gamepad1.left_stick_y) > .1 || Math.abs(gamepad1.right_stick_x) > .1) {
            testDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_trigger, angleLock, lockedAngle);
        } else {
            stopMotors();
        }
    }
}
