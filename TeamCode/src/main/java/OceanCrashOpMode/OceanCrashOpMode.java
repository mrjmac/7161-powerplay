package OceanCrashOpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class OceanCrashOpMode extends OpMode {

    // Drivetrain
    private DcMotor BL; // [E1]
    private DcMotor BR; // [C0]
    private DcMotor FL; // [E0]
    private DcMotor FR; // [C1]

    // Four Bar
    private Servo spinL; // [C0]
    private Servo spinR; // [E5]
    private Servo grab; // [E4]

    // Intake
    private DcMotor intakeL; // [E2]
    private DcMotor intakeR; // [C2]

    // Lift
    private DcMotor liftL; // [E3]
    private DcMotor liftR; // [C3]

    public enum LiftState {
        IDLE,
        BEACON,
        RAISE,
        PLACE,
        LOWER,
    }
    public int jHeight = 3;
    public int liftTargetPos = 0;

    public boolean active = false;
    public boolean grabbed = false;

    public ElapsedTime macroTime = new ElapsedTime();
    public ElapsedTime grabTime = new ElapsedTime();
    public ElapsedTime jHeightTime = new ElapsedTime();


    LiftState lift = LiftState.IDLE;

    public void init() {


        // Drivetrain
        FR = hardwareMap.dcMotor.get("FR"); // [C1]
        FL = hardwareMap.dcMotor.get("FL"); // [E0]
        BR = hardwareMap.dcMotor.get("BR"); // [C0]
        BL = hardwareMap.dcMotor.get("BL"); // [E1]

        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Four Bar
        spinL = hardwareMap.servo.get("spinL"); // [C0]
        spinR = hardwareMap.servo.get("spinR"); // [E5]
        grab = hardwareMap.servo.get("grab"); // [E4]

        // Intake
        intakeL = hardwareMap.dcMotor.get("intakeL"); // [E2]
        intakeR = hardwareMap.dcMotor.get("intakeR"); // [C2]

        intakeL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeL.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeL.setPower(0);

        intakeR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeR.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeR.setPower(0);


        // Lift
        liftL = hardwareMap.dcMotor.get("liftL"); // [E3]
        liftR = hardwareMap.dcMotor.get("liftR"); // [C3]

        liftL.setDirection(DcMotorSimple.Direction.FORWARD);
        liftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftR.setDirection(DcMotorSimple.Direction.REVERSE);
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

    public void drive(double x, double y, double turn, double trigger) {

        double FLP = y - turn - x;
        double FRP = y + turn + x;
        double BLP = y + turn - x;
        double BRP = y - turn + x;

        double speedControl;
        if (trigger > .1) {
            speedControl = .35;
        } else {
            speedControl = 1;
        }

        double max = Math.max(Math.max(Math.abs(FLP), Math.abs(FRP)), Math.max(Math.abs(BLP), Math.abs(BRP)));

        if (max > 1) {
            FLP /= max;
            FRP /= max;
            BLP /= max;
            BRP /= max;
        }

        startMotors(FLP * speedControl, FRP * speedControl, BLP * speedControl, BRP * speedControl);

        telemetry.addData("FLP: ", FLP * speedControl);
        telemetry.addData("FRP: ", FRP * speedControl);
        telemetry.addData("BLP: ", BLP * speedControl);
        telemetry.addData("BRP: ", BRP * speedControl);
        telemetry.addData("right trigger: ", gamepad1.right_trigger);
    }


    public void setIntake(double p)
    {
        intakeL.setPower(p);
        intakeR.setPower(-p);
    }

    public void extendFourBar()
    {
        spinL.setPosition(0.3);
        spinR.setPosition(0.7);
    }

    public void retractFourBar()
    {
        spinR.setPosition(0);
        spinL.setPosition(1);
    }

    public void grab()
    {
        grab.setPosition(1);
    }

    public void release()
    {
        grab.setPosition(.92);
    }


    public double getLiftPos()
    {
        return Math.abs((liftL.getCurrentPosition() + liftR.getCurrentPosition()) / 2.0);
    }

    public void setLiftPower(double power)
    {
        liftL.setPower(power);
        liftR.setPower(power);
    }

    public void resetLiftEncoder()
    {
        liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setLift(double liftTargetPos)
    {
        if (getLiftPos() <= liftTargetPos - 50)
        {
            setLiftPower(1);
        }
    }

    public void liftReset(double power)
    {
        if (getLiftPos() >= 50)
        {
            setLiftPower(power);
        }
    }

}
