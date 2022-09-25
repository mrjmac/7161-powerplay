package OceanCrashOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp", group = "opMode")
public class OceanCrashTeleOp extends OceanCrashOpMode{

    public void loop() {

        // DRIVE
        if (Math.abs(gamepad1.left_stick_x) > 1 || Math.abs(gamepad1.left_stick_y) > .1 || Math.abs(gamepad1.right_stick_x) > .1)
        {
            drive(gamepad1.right_stick_x, gamepad1.left_stick_y, gamepad1.left_stick_x);
        }
        else
        {
            stopMotors();
        }

        // INTAKE
        if (gamepad1.right_trigger > .1)
        {
            setIntake(1);
        }
        else if (gamepad1.left_trigger > .1)
        {
            setIntake(-1);
        }
        else
        {
            setIntake(0);
        }
        // FOUR BAR
        if (gamepad2.left_trigger > .1)
        {
            retractFourBar();
        }
        if (gamepad2.right_trigger > .1)
        {
            extendFourBar();
        }
        if (gamepad2.a)
        {
            grab();
        }
        if (gamepad2.b)
        {
            release();
        }


    }
}
