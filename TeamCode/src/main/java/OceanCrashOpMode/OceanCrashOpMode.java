package OceanCrashOpMode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import OceanCrashRoadrunner.drive.SampleMecanumDrive;

public abstract class OceanCrashOpMode extends OpMode {

    // Drivetrain
    private DcMotor BL; // [E0]
    private DcMotor BR; // [C0]
    private DcMotor FL; // [E1]
    private DcMotor FR; // [C1], lateral odom

    // Four Bar
    private Servo spinL1; // [C0]
    private Servo spinL2; // [C5]
    private Servo spinR1; // [E5]
    private Servo spinR2; // [C4]
    private Servo grab; // [E4]
    private Servo swivel;

    // Intake
    private DcMotor intakeL; // [E2], left odom
    private DcMotor intakeR; // [C2], right odom
    private ColorSensor colorS; //[]

    // Lift
    private DcMotor liftL; // [E3]
    private DcMotor liftR; // [C3]
    private boolean grabLift = false;

    private TouchSensor touch;

    private ElapsedTime jit;
    public double currentTargetSlidesPos = 0, pastError = 0, pastTime = 0;
    public static double kP = .00466666667, kD = 0.2;

    private Orientation angles;
    private BNO055IMU imu;


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

        // Four Bar
        spinL1 = hardwareMap.servo.get("spinL1"); // [C0]
        spinL2 = hardwareMap.servo.get("spinL2"); // []
        spinR1 = hardwareMap.servo.get("spinR1"); // [E5] //these are wrong
        spinR2 = hardwareMap.servo.get("spinR2"); // [C0]
        grab = hardwareMap.servo.get("grab"); // [E4]
        swivel = hardwareMap.servo.get("swivel");

        // Intake
        intakeL = hardwareMap.dcMotor.get("intakeL"); // [E2]
        intakeR = hardwareMap.dcMotor.get("intakeR"); // [C2]
        colorS = hardwareMap.get(ColorSensor.class, "color");


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

        touch = hardwareMap.touchSensor.get("touch"); // [E0]

        liftL.setDirection(DcMotorSimple.Direction.FORWARD);
        liftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftR.setDirection(DcMotorSimple.Direction.REVERSE);
        liftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu2");
        imu.initialize(parameters);

        jit = new ElapsedTime();
        release();


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

        double FLP = y - x - turn;
        double FRP = y + x + turn;
        double BLP = y + x - turn;
        double BRP = y - x + turn;

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
        double voltageC = 1;
        if (turn > 0)
            if (getVoltage() > 14)
                voltageC = .8;
            else if (getVoltage() > 13.8)
                voltageC = .85;
            else if (getVoltage() > 13.6)
                voltageC = .9;
            else if (getVoltage() > 13.4)
                voltageC = .95;
        startMotors(FLP * speedControl * voltageC, FRP * speedControl * voltageC, BLP * speedControl * voltageC, BRP * speedControl * voltageC);

        telemetry.addData("FLP: ", FLP * speedControl);
        telemetry.addData("FRP: ", FRP * speedControl);
        telemetry.addData("BLP: ", BLP * speedControl);
        telemetry.addData("BRP: ", BRP * speedControl);
        telemetry.addData("right trigger: ", gamepad1.right_trigger);
    }

    public void testDrive(double x, double y, double turn, double trigger, boolean angleLock, double lockedAngle) {

        double FLP = y - x - turn;
        double FRP = y + x + turn;
        double BLP = y + x - turn;
        double BRP = y - x + turn;

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
        double voltageC = 1;
        if (!angleLock) {
            if (Math.abs(turn) > .1)
                if (getVoltage() > 14)
                    voltageC = .8;
                else if (getVoltage() > 13.8)
                    voltageC = .85;
                else if (getVoltage() > 13.6)
                    voltageC = .9;
                else if (getVoltage() > 13.4)
                    voltageC = .95;
        } else {
            if (Math.abs(x) > .05 && Math.abs(y) > .05) {
                double angleDiff = gimbleCalc(lockedAngle, getGyroYaw());
                double GyroScalePower = angleDiff * .1;
                BRP -= GyroScalePower;
                FRP -= GyroScalePower;
                BLP += GyroScalePower;
                FLP += GyroScalePower;
            }
        }

        startMotors(FLP * speedControl * voltageC, FRP * speedControl * voltageC, BLP * speedControl * voltageC, BRP * speedControl * voltageC);

        telemetry.addData("FLP: ", FLP * speedControl);
        telemetry.addData("FRP: ", FRP * speedControl);
        telemetry.addData("BLP: ", BLP * speedControl);
        telemetry.addData("BRP: ", BRP * speedControl);
        telemetry.addData("right trigger: ", gamepad1.right_trigger);
        telemetry.addData("angleLock:", angleLock);
        telemetry.addData("lockedAngle:", lockedAngle);
    }

    public void updateGyroValues() {
        angles = imu.getAngularOrientation();
    }

    public double getGyroYaw() {
        updateGyroValues();
        return angles.firstAngle;
    }

    public double gimbleCalc(double initVal, double curVal) {
        double angleDiff;
        if (initVal - curVal >= 180)
            angleDiff = -1 * ((360 + curVal) - initVal);
        else if (curVal - initVal >= 180)
            angleDiff = (360 + initVal) - curVal;
        else
            angleDiff = initVal - curVal;

        return angleDiff;
    }

    public void setIntake(double p)
    {
        intakeL.setPower(p);
        intakeR.setPower(-p);
    }

    public double getVoltage()
    {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
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
        spinR1.setPosition(0.5625);
        spinR2.setPosition(0.5625);
        spinL1.setPosition(1-.5625);
        spinL2.setPosition(1-.5625);
    }

    public void dropFourBar() {
        spinR1.setPosition(0.50);
        spinR2.setPosition(0.50);
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
        swivelIn();
        grab();
        spinR1.setPosition(1);
        spinR2.setPosition(1);
        spinL1.setPosition(0);
        spinL2.setPosition(0);
    }

    public void grabFourBar()
    {
        spinR1.setPosition(0.9);
        spinR2.setPosition(0.9);
        spinL1.setPosition(0.1);
        spinL2.setPosition(0.1);
    }

    public void grab()
    {
        grab.setPosition(0.255);
    }

    public void release()
    {
        grab.setPosition(.52);  //tune this
    }

    public void swivelIn()
    {
        swivel.setPosition(0.025);
    }

    public void swivelOut()
    {
        swivel.setPosition(.67);
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

    public void setLiftPos(double liftTargetPos)
    {
        if (getLiftPos() <= liftTargetPos - 50)
        {
            setLiftPower(-.9);
        }
    }

    public void setLiftPosLittle(double liftTargetPos)
    {
        if (getLiftPos() <= liftTargetPos)
        {
            setLiftPower(-.2);
        }
    }

    public void liftReset(double power, double liftTargetPos)
    {

        double p = Math.abs(liftTargetPos - Math.abs(getLiftPos())) / 250.0;

        if (getLiftPos() - 50 < 0)
        {
            setLiftPower(power);
        }
        else if (getLiftPos() > liftTargetPos)
        {
            setLiftPower(Math.min(power, power * p));
        }
    }

    public boolean grabBlue()
    {
        return colorS.red() < 35 && colorS.green() < 50 && colorS.blue() > 52;
    }

    public boolean grabRed()
    {
        return colorS.red() > 75 && colorS.green() < 55 && colorS.blue() < 50;
    }

    public boolean isTouch()
    {
        return touch.isPressed();
    }

    public double getPower()
    {
        return (liftL.getPower() + liftR.getPower()) / 2.0;
    }

    public double getLiftL()
    {
        return liftL.getCurrentPosition();
    }

    public double getLiftR()
    {
        return liftR.getCurrentPosition();
    }

    public void setSlideTarget(double target, boolean grab) {
        currentTargetSlidesPos = target;
        grabLift = grab;
    }

    public void updateLiftLength(double liftTime) {
        double error = currentTargetSlidesPos - getLiftPos();
        if (Math.abs(error) > .5) {
            if (!grabLift) {
                double p = Math.signum(error) * Math.sqrt((Math.abs(error)) * kP) * ((13.2 / getVoltage()) * 1);

                double dT = liftTime - pastTime;
                double d = Math.signum(error - pastError) * Math.sqrt(Math.abs(error - pastError) / dT * kD);

                if (Math.abs(error) < 1.6) {
                    if (getLiftPos() > 20) {
                        p = 0.0093 / (13.8 / getVoltage());
                        d = 0;
                    } else {
                        p = 0;
                        d = 0;
                    }
                } else if (error > 250) {
                    p *= 1;
                } else if (error > 100) {
                    p *= .8;
                } else if (error > 30 && getLiftPos() < currentTargetSlidesPos) {
                    d *= 1.5;
                } else if (error < 30 && error > 0 && getLiftPos() > 700) {
                    p *= .65;
                    d /= 1.4;
                } else if (error < 30 && error > 0 && getLiftPos() < 700) {
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
                if (liftTime < 3000) {
                    power *= .4;
                }
                setLiftPower(-power);
            } else {
                double p = Math.signum(error) * Math.sqrt((Math.abs(error)) * kP);
                setLiftPower(-p * 1.3);
            }
        }
        pastTime = liftTime;
        pastError = error;
    }
}
