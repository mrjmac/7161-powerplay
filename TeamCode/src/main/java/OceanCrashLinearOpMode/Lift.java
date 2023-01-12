package OceanCrashLinearOpMode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Config
public class Lift {

    private final DcMotor liftL; // [E3]
    public final DcMotor liftR; // [C3]

    private final LinearOpMode opMode;

    private TouchSensor touch;

    private double power = .7;

    private boolean LBat = true;

    public Servo spinL; // [C0]
    public Servo spinR; // [E5]
    private Servo grab; // [E4]
    private Servo swivel;


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
        swivel = this.opMode.hardwareMap.servo.get("swivel");

        touch = this.opMode.hardwareMap.touchSensor.get("touch");

        if (getVoltage() > 14.5)
        {
            power = .4;

        }
        else
        {
            power = .7;
        }

        resetEncoder();
        grab();
        swivelIn();
        //swivelOut();
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
        if (Math.abs(liftTargetPos - getLiftPos()) > 35 && this.opMode.opModeIsActive()) {
            this.opMode.telemetry.addData("lift :: ", getLiftPos());
            this.opMode.telemetry.addData("error :: ", getLiftPos() - liftTargetPos);
            this.opMode.telemetry.update();
            if (liftTargetPos < getLiftPos())
                setLiftPower(power);
            else
                setLiftPower(-power);


        } else if (liftTargetPos == 0)
            setLiftPower(0);
        else
            setLiftPower(STALL_POWER);

    }


    public void tuneLiftPos(double liftTargetPos, double up, double down)
    {
        if (Math.abs(liftTargetPos - getLiftPos()) > 100 && this.opMode.opModeIsActive()) {
            this.opMode.telemetry.addData("lift :: ", getLiftPos());
            this.opMode.telemetry.addData("error :: ", getLiftPos() - liftTargetPos);
            this.opMode.telemetry.update();
            if (liftTargetPos < getLiftPos())
                setLiftPower(up);
            else
                setLiftPower(down);


        } else if (liftTargetPos == 0)
            setLiftPower(0);
        else
            setLiftPower(STALL_POWER);

    }

    public double getVoltage()
    {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : this.opMode.hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
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
        spinR.setPosition(0.5);
        spinL.setPosition(0.5);
    }

    public void trueExtendFourBar()
    {
        spinR.setPosition(0.65);
        spinL.setPosition(0.35);
    }

    public void lastCycleFourBar()
    {
        spinR.setPosition(0.75);
        spinL.setPosition(0.25);
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
        grab.setPosition(0.1);
    }

    public void release()
    {
        grab.setPosition(.43);  //tune this
    }

    public void swivelIn()
    {
        swivel.setPosition(0);
    }

    public void swivelOut()
    {
        swivel.setPosition(.67);
    }

    public void swivelStartLeft()
    {
        swivel.setPosition(.43);
    }

    public void swivelStartRight()
    {
        swivel.setPosition(.83);
    }

    public boolean isTouch()
    {
        return touch.isPressed();
    }
}
