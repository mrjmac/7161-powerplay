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

    private Servo spinL; // [C0]
    private Servo spinR; // [E5]
    private Servo grab; // [E4]

    private final double lowTicks = 200;
    private final double medTicks = 400;
    private final double highTicks = 600;
    private final double STALL_POWER = .09;

    public Lift(LinearOpMode opMode) throws InterruptedException {

        this.opMode = opMode;

        liftL = this.opMode.hardwareMap.dcMotor.get("liftL"); // [E3]
        liftR = this.opMode.hardwareMap.dcMotor.get("liftR"); // [C3]

        liftL.setDirection(DcMotorSimple.Direction.REVERSE);
        liftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftR.setDirection(DcMotorSimple.Direction.FORWARD);
        liftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        spinL = this.opMode.hardwareMap.servo.get("spinL"); // [C0]
        spinR = this.opMode.hardwareMap.servo.get("spinR"); // [E5]
        grab = this.opMode.hardwareMap.servo.get("grab"); // [E4]

        grab.setPosition(1);
        spinR.setPosition(0);
        spinL.setPosition(0);

    }

    public void setPower(double power)
    {
        liftL.setPower(power);
        liftR.setPower(power);
    }

    public int getEncoderAvg() {
        return Math.abs(liftL.getCurrentPosition() + liftR.getCurrentPosition()) / 2;
    }

    public void resetEncoder() {
        liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setLift(int height, double p)
    {
        //low == 1, med == 2, high == 3
        double ticks, kP = p / 100;

        if (height == 1) {
            ticks = lowTicks;
        }
        else if (height == 2) {
            ticks = medTicks;
        }
        else {
            ticks = highTicks;
        }

        while (this.opMode.opModeIsActive() && this.opMode.isStopRequested()) {

            while (getEncoderAvg() <= ticks && this.opMode.opModeIsActive()) {
                double error = (ticks - getEncoderAvg());
                double changeP = error * kP;

                setPower(changeP);

                if (error < 10 || Math.abs(changeP) < 0.2) {
                    setPower(STALL_POWER);
                    break;
                }
            }
        }

        grab.setPosition(0);
    }

    public void setLiftForCone(int cycleNum, double p)
    {
        //low == 1, med == 2, high == 3
        double ticks = 500, kP = p / 100;

        ticks -= cycleNum * 50;

        while (this.opMode.opModeIsActive() && this.opMode.isStopRequested()) {

            while (getEncoderAvg() <= ticks && this.opMode.opModeIsActive()) {
                double error = (ticks - getEncoderAvg());
                double changeP = error * kP;

                setPower(changeP);

                if (error < 10 || Math.abs(changeP) < 0.2) {
                    setPower(STALL_POWER);
                    break;
                }
            }
        }

        grab.setPosition(1);

    }

    public void resetLift(double p)
    {
        while (getEncoderAvg() > 0){
            double power = getEncoderAvg() * p;
            setPower(-power);
            if (Math.abs(power) < .05){
                break;
            }
        }
        setPower(0);

        // TODO: Figure out if we call resetEncoder()
    }

    public void extendFourBar()
    {
        grab.setPosition(1);
        spinL.setPosition(1);
        spinR.setPosition(-1);
    }
}
