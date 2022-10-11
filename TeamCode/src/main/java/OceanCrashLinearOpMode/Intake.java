package OceanCrashLinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Intake {

    private DcMotor intakeL; // [E2]
    private DcMotor intakeR; // [C2]

    private LinearOpMode opMode;

    public Intake(LinearOpMode opMode) throws InterruptedException {

        this.opMode = opMode;

        intakeL = this.opMode.hardwareMap.dcMotor.get("intakeL"); // [E2]
        intakeR = this.opMode.hardwareMap.dcMotor.get("intakeR"); // [C2]

        intakeL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeL.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeL.setPower(0);

        intakeR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeR.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeR.setPower(0);
    }

    public void startIntake(double power) {
        intakeL.setPower(power);
        intakeR.setPower(power);
    }

    public void resetIntakeEncoder() {
        intakeL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
