package OceanCrashLinearOpMode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.BufferedInputStream;

@Config
public class Lift {

    private final DcMotor liftL; // [E2]
    public final DcMotor liftR; // [C2]

    private final LinearOpMode opMode;

    private TouchSensor touch;

    private double power = .7;

    private boolean LBat = true;

    public Servo spinL1; // [C0] // this might be right
    public Servo spinL2; // []
    public Servo spinR1; // [E5] // this is wrong
    public Servo spinR2; // []
    private Servo grab; // [C2] // this might be right
    private Servo swivel;


    private final double STALL_POWER = -0.0005;

    public double currentTargetSlidesPos = 0, pastError = 0, pastTime = 0;
    public static double kP = .00466666667, kD = 0.0925, kStatic = -0.0005;
    private ElapsedTime liftTime = new ElapsedTime();


    public Lift(LinearOpMode opMode) throws InterruptedException {

        this.opMode = opMode;

        liftL = this.opMode.hardwareMap.dcMotor.get("liftL"); // [E2]
        liftR = this.opMode.hardwareMap.dcMotor.get("liftR"); // [C2]

        liftL.setDirection(DcMotorSimple.Direction.FORWARD);
        liftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftR.setDirection(DcMotorSimple.Direction.REVERSE);
        liftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        spinL1 = this.opMode.hardwareMap.servo.get("spinL1"); // [C0]
        spinL2 = this.opMode.hardwareMap.servo.get("spinL2"); // []
        spinR1 = this.opMode.hardwareMap.servo.get("spinR1"); // [E5] //these are wrong
        spinR2 = this.opMode.hardwareMap.servo.get("spinR2"); // [C0]
        grab = this.opMode.hardwareMap.servo.get("grab"); // [E4]
        swivel = this.opMode.hardwareMap.servo.get("swivel");

        touch = this.opMode.hardwareMap.touchSensor.get("touch");

        if (getVoltage() > 14.2)
        {
            power = .46;

        }
        else if (getVoltage() > 13.8)
        {
            power = .5;
        }
        else if (getVoltage() > 13.4)
        {
            power = .5;
        }
        else if (getVoltage() > 13.1)
        {
            power = .55;
        }
        else
        {
            power = .725;
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

    //TODO: SPINR and SPINL NEED TO SWAP STARTING EXTREMES; DRIVEN THEORETICALLY HAS 450deg ROM, FIX PROPORTIONS
    public void extendFourBarStart() {
        grabStart();
        spinR1.setPosition(0.5);
        spinR2.setPosition(0.5);
        spinL1.setPosition(0.5);
        spinL2.setPosition(0.5);
    }

    public void extendFourBar()
    {
        grab();
        spinR1.setPosition(0.5);
        spinR2.setPosition(0.5);
        spinL1.setPosition(0.5);
        spinL2.setPosition(0.5);
    }

    public void trueExtendFourBar()
    {
        spinR1.setPosition(0.35);
        spinR2.setPosition(0.35);
        spinL1.setPosition(0.65);
        spinL2.setPosition(0.65);
    }

    public void retractFourBar() //NEW R is 1; OLD R is 0
    {
        grab();
        spinR1.setPosition(1);
        spinR2.setPosition(1);
        spinL1.setPosition(0);
        spinL2.setPosition(0);
    }

    public void neutralFourBar() //NEW R is 1; OLD R is 0
    {
        spinR1.setPosition(.75);
        spinR2.setPosition(.75);
        spinL1.setPosition(.25);
        spinL2.setPosition(.25);
    }

    //TODO: SPINR and SPINL NEED TO SWAP STARTING EXTREMES; DRIVEN THEORETICALLY HAS 450deg ROM, FIX PROPORTIONS


    public void grabStart() {grab.setPosition(.255);} //proposed parameter for start

    public void grab()
    {
        grab.setPosition(0.315);
    } //.305 for grabStack

    public void release()
    {
        grab.setPosition(.52);  //tune this
    }

    public void swivelIn()
    {
        swivel.setPosition(0);
    }

    public void swivelOut()
    {
        swivel.setPosition(.67);
    }

    public void swivelDeposit()
    {
        swivel.setPosition(.74);
    }

    public void swivelStartLeft()
    {
        swivel.setPosition(.36);
    }

    public void swivelStartRight()
    {
        swivel.setPosition(.93);
    }

    public boolean isTouch()
    {
        return touch.isPressed();
    }

    public double liftTickstoInches(double ticks) {
        return ticks / 145.1 * (1.5 * Math.PI);
    }

    public void setSlideTarget(double target) {
        currentTargetSlidesPos = target;
    }

    public double getSlidesPos() {
        return getLiftPos();
    }

    public void updateLiftLength(double liftTime) {
        double error = currentTargetSlidesPos - getLiftPos();

        if (Math.abs(error) > .5) {
            double p = Math.signum(error) * Math.sqrt((Math.abs(error)) * kP);

            double dT = liftTime - pastTime;
            double d = Math.signum(error - pastError) * Math.sqrt(Math.abs(error - pastError) / dT * kD);

            double f = 0;
            //might want to add if statements to fine tune very important movements/if in acceptable error, stallPower
            if (Math.abs(error) < 2) {
                //f = getSlidesPos() * kStatic;
                if (getLiftPos() > 20) {
                    p = 0.0005;
                    d = 0;
                } else {
                    p = 0;
                    d = 0;
                }
            } else if (error > 250) {
                p *= .8;
            } else if (error > 100) {
                p *= 1;
            } else if (error > 30 && getLiftPos() < currentTargetSlidesPos) {
                d *= 1.5;
            } else if (error < 30 && error > 0 && liftTime > 1500) {
                p *= .4;
            } else if (getLiftPos() > currentTargetSlidesPos) {
                if (error > 30) {
                    p /= 1.2;
                    //d *= .099705882;
                } else {
                    p *= .5;
                    //d = 0;
                }
                d *= .099705882;
            }
            double power = p + d;
            setLiftPower(-power);
        }
        pastTime = liftTime;
        pastError = error;
    }
}
