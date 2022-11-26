package OceanCrashLinearOpMode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Lift {

    private final DcMotor liftL; // [E3]
    private final DcMotor liftR; // [C3]

    private final LinearOpMode opMode;

    public Servo spinL; // [C0]
    public Servo spinR; // [E5]
    private final Servo grab; // [E4]

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

    public int getLiftL() {
        return liftL.getCurrentPosition();
    }

    public int getLiftR() {
        return liftR.getCurrentPosition();
    }

    public void resetEncoder() {
        liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setLiftPos(double liftTargetPos)
    {
        if (Math.abs(liftTargetPos - getLiftPos()) > 100 && this.opMode.opModeIsActive()) {
            this.opMode.telemetry.addData("lift :: ", getLiftPos());
            this.opMode.telemetry.addData("error :: ", getLiftPos() - liftTargetPos);
            this.opMode.telemetry.update();
            if (liftTargetPos < getLiftPos())
                setLiftPower(.6);
            else
                setLiftPower(-.8);


        } else
            if (liftTargetPos == 0)
                setLiftPower(0);
            else
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

    public void startFourBar()
    {
        grab();
        spinR.setPosition(0.6);
        spinL.setPosition(0.4);
    }

    public void retractFourBar()
    {
        grab();
        spinR.setPosition(0);
        spinL.setPosition(1);
    }

    public void grab()
    {
        grab.setPosition(.85);
    }

    public void release()
    {
        grab.setPosition(.7);
    }
}
