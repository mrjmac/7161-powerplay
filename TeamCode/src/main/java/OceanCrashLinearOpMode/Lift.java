package OceanCrashLinearOpMode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Lift {

    private DcMotor liftL; // [E3]
    private DcMotor liftR; // [C3]

    private LinearOpMode opMode;

    public Servo spinL; // [C0]
    public Servo spinR; // [E5]
    private Servo grab; // [E4]

    private final double STALL_POWER = -0.0005;

    public Lift(LinearOpMode opMode) throws InterruptedException {

        this.opMode = opMode;

        liftL = this.opMode.hardwareMap.dcMotor.get("liftL"); // [E3]
        liftR = this.opMode.hardwareMap.dcMotor.get("liftR"); // [C3]

        liftL.setDirection(DcMotorSimple.Direction.FORWARD);
        liftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftR.setDirection(DcMotorSimple.Direction.REVERSE);
        liftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        spinL = this.opMode.hardwareMap.servo.get("spinL"); // [C0]
        spinR = this.opMode.hardwareMap.servo.get("spinR"); // [E5]
        grab = this.opMode.hardwareMap.servo.get("grab"); // [E4]

        resetEncoder();

    }

    public void setLiftPower(double power)
    {
        liftL.setPower(power);
        liftR.setPower(power);
    }

    public int getLiftPos() {
        return Math.abs(liftL.getCurrentPosition() + liftR.getCurrentPosition()) / 2;
    }

    public void resetEncoder() {
        liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setLiftPos(double liftTargetPos)
    {
        if (Math.abs(liftTargetPos - getLiftPos()) > 150 && this.opMode.opModeIsActive()) {
            this.opMode.telemetry.addData("lift :: ", getLiftPos());
            this.opMode.telemetry.addData("error :: ", getLiftPos() - liftTargetPos);
            this.opMode.telemetry.update();
            if (liftTargetPos < getLiftPos())
                setLiftPower(.3);
            else
                setLiftPower(-.5);


        } else
        setLiftPower(STALL_POWER);

    }

    public void resetLift(double p, int targetPos)
    {
        while (getLiftPos() >= targetPos && this.opMode.opModeIsActive()){
            setLiftPower(p);
            if (getLiftPos() <= targetPos)
            {
                setLiftPower(0);
                break;
            }
        }
    }

    public void extendFourBar()
    {
        grab();
        spinR.setPosition(0.7);
        spinL.setPosition(0.3);
    }

    public void retractFourBar()
    {
        spinR.setPosition(0);
        spinL.setPosition(1);
    }

    public void grab()
    {
        grab.setPosition(.9);
    }

    public void release()
    {
        grab.setPosition(.7);
    }
}
