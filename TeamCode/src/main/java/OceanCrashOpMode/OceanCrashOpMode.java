package OceanCrashOpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class OceanCrashOpMode extends OpMode {

    // Drivetrain
    private DcMotor BL;
    private DcMotor BR;
    private DcMotor FL;
    private DcMotor FR;

    // Four Bar
    private Servo spinL;
    private Servo spinR;
    private Servo grab;

    // Intake
    private DcMotor intakeL;
    private DcMotor intakeR;

    // Lift
    private DcMotor liftL;
    private DcMotor liftR;

    public void init() {

        // Drivetrain
        FR = hardwareMap.dcMotor.get("FR");
        FL = hardwareMap.dcMotor.get("FL");
        BR = hardwareMap.dcMotor.get("BR");
        BL = hardwareMap.dcMotor.get("BL");

        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Four Bar
        spinL = hardwareMap.servo.get("spinL");
        spinR = hardwareMap.servo.get("spinR");
        grab = hardwareMap.servo.get("grab");

        // Intake
        intakeL = hardwareMap.dcMotor.get("intakeL");
        intakeR = hardwareMap.dcMotor.get("intakeR");

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

        // Lift
        liftL = hardwareMap.dcMotor.get("liftL");
        liftR = hardwareMap.dcMotor.get("liftR");

        liftL.setDirection(DcMotorSimple.Direction.REVERSE);
        liftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftR.setDirection(DcMotorSimple.Direction.FORWARD);
        liftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("init ", "completed");
        telemetry.update();
    }

    public void startMotors(double FLP, double FRP, double BLP, double BRP) {
        FL.setPower(FLP);
        FR.setPower(FRP);
        BL.setPower(BLP);
        BR.setPower(BRP);
    }

    public void stopMotors()
    {
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    public void drive(double x, double y, double turn) {

        double FLP = y - turn - x;
        double FRP = y + turn + x ;
        double BLP = y + turn - x;
        double BRP = y - turn + x;

        double max = Math.max(Math.max(Math.abs(FLP), Math.abs(FRP)), Math.max(Math.abs(BLP), Math.abs(BRP)));

        if (max > 1) {
            FLP /= max;
            FRP /= max;
            BLP /= max;
            BRP /= max;
        }

        if (gamepad1.right_trigger > .1) {
            FLP *= .35;
            FRP *= .35;
            BLP *= .35;
            BRP *= .35;
        }

        startMotors(FLP, FRP, BLP, BRP);
    }

    public void setIntake(double p)
    {
        intakeL.setPower(p);
        intakeR.setPower(-p);
    }

    public void extendFourBar()
    {
        spinL.setPosition(1);
        spinR.setPosition(-1);
    }

    public void retractFourBar()
    {
        spinR.setPosition(0);
        spinL.setPosition(0);
    }

    public void grab()
    {
        grab.setPosition(1);
    }

    public void release()
    {
        grab.setPosition(0);
    }
}
