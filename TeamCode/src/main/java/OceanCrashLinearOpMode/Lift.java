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
    private final DcMotor liftR; // [C2]

    private final LinearOpMode opMode;

    private TouchSensor touch;

    private double power = .7;

    private Servo spinL1; // [C0]
    private Servo spinL2; // []
    private Servo spinR1; // [E5]
    private Servo spinR2; // []
    private Servo grab; // [C2]
    private Servo swivel;


    private final double STALL_POWER = -0.0005;

    private double currentTargetSlidesPos = 0, pastError = 0, pastTime = 0;
    private static double kP = .00466666667, kD = 0.0925, kStatic = -0.0005;
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
        spinR1 = this.opMode.hardwareMap.servo.get("spinR1"); // [E5]
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

    public void retractFourBar()
    {
        grab();
        spinR1.setPosition(1);
        spinR2.setPosition(1);
        spinL1.setPosition(0);
        spinL2.setPosition(0);
    }

    public void neutralFourBar()
    {
        spinR1.setPosition(.75);
        spinR2.setPosition(.75);
        spinL1.setPosition(.25);
        spinL2.setPosition(.25);
    }

    public void grab()
    {
        grab.setPosition(.275);
    }

    public void release() {
        grab.setPosition(.55);
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
        swivel.setPosition(.36);
    }

    public void swivelStartRight()
    {
        swivel.setPosition(.93);
    }

    public void setSlideTarget(double target) {
        currentTargetSlidesPos = target;
    }


    public void updateLiftLength(double liftTime) {
        double error = currentTargetSlidesPos - getLiftPos();

        if (Math.abs(error) > .5) {
            double p = Math.signum(error) * Math.sqrt((Math.abs(error)) * kP);

            double dT = liftTime - pastTime;
            double d = Math.signum(error - pastError) * Math.sqrt(Math.abs(error - pastError) / dT * kD);

            if (Math.abs(error) < 2) {
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
                } else {
                    p *= .5;
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
