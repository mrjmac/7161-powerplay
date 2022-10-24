package OceanCrashLinearOpMode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.Angle;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Drivetrain {

    private DcMotor BL; // [E1]
    private DcMotor BR; // [C0]
    private DcMotor FL; // [E0]
    private DcMotor FR; // [C1]

    private LinearOpMode opMode;
    private final String LOG_TAG = "DriveTrain";
    private ElapsedTime runtime = new ElapsedTime();

    private Orientation angles;
    private BNO055IMU imu;

    public Drivetrain(LinearOpMode opMode) throws InterruptedException {

        this.opMode = opMode;
        FR = this.opMode.hardwareMap.dcMotor.get("FR"); // [C1]
        FL = this.opMode.hardwareMap.dcMotor.get("FL"); // [E0]
        BR = this.opMode.hardwareMap.dcMotor.get("BR"); // [C0]
        BL = this.opMode.hardwareMap.dcMotor.get("BL"); // [E1]

        this.opMode.telemetry.addData(LOG_TAG + "init", "finished init");
        this.opMode.telemetry.update();

        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = this.opMode.hardwareMap.get(BNO055IMU.class, "imu2");
        imu.initialize(parameters);
    }

    public void resetEncoders() {
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void rightGyroStrafe(double speed, double inches, double timeoutS, double heading) {

        double ticks = inches * 32.4;
        heading = -heading;
        //runtime isn't used, this is just a backup call which we don't need
        double kP = speed / (3.3);
        resetEncoders();
        runtime.reset();

        while (Math.abs(getEncoderAvg()) < ticks && this.opMode.opModeIsActive()) {

            double error = (ticks - Math.abs(getEncoderAvg())) / 32;
            double ChangeP = kP * error;

            if (ChangeP > 1)
                ChangeP /= ChangeP;
            else if (ChangeP < .23) {
                double x = .23/ChangeP;
                ChangeP *= x;
            }

            double angleDiff = GimbleCalc(heading, getGyroYaw());
            double GyroScalePower = angleDiff * .04;

            if (ChangeP > 1){
                ChangeP /= ChangeP;
            }

            //If this function doesn't work, this is probably why
            if(angleDiff > 2) {
                BL.setPower(ChangeP);
                BR.setPower(ChangeP);
                FL.setPower(ChangeP - GyroScalePower);
                FR.setPower(ChangeP - GyroScalePower);
            } else if (angleDiff < -2) {
                BL.setPower(ChangeP - GyroScalePower);
                BR.setPower(ChangeP - GyroScalePower);
                FL.setPower(ChangeP);
                FR.setPower(ChangeP);
            } else {
                BL.setPower(ChangeP);
                BR.setPower(ChangeP);
                FL.setPower(ChangeP);
                FR.setPower(ChangeP);
            }

            if (error < .25 || runtime.seconds() > timeoutS || Math.abs(ChangeP) < .15) {
                stopMotors();
                break;
            }

            this.opMode.telemetry.addData("horizontal:", getEncoderAvg());
            this.opMode.telemetry.addData("YawAngle:", getGyroYaw());
            this.opMode.telemetry.update();
        }
        stopMotors();
    }

    public void leftGyroStrafe(double speed, double inches, double timeoutS, double heading) {

        double ticks = inches * 32.4;
        heading = -heading;
        //runtime isn't used, this is just a backup call which we don't need
        double kP = speed / 3.3;
        resetEncoders();
        runtime.reset();

        while (Math.abs(getEncoderAvg()) < ticks && this.opMode.opModeIsActive()) {

            double error = (ticks - Math.abs(getEncoderAvg())) / 32;
            double ChangeP = kP * error;

            if (ChangeP > 1)
                ChangeP /= ChangeP;
            else if (ChangeP < .23) {
                double x = .23/ChangeP;
                ChangeP *= x;
            }

            double angleDiff = GimbleCalc(heading, getGyroYaw());
            double GyroScalePower = angleDiff * .04;

            if (ChangeP > 1){
                ChangeP /= ChangeP;
            }

            //if this function doesn't work, this is is probably why
            if (angleDiff > 2) {
                BR.setPower(ChangeP + GyroScalePower);
                BL.setPower(ChangeP + GyroScalePower);
                FL.setPower(ChangeP);
                FR.setPower(ChangeP);
            } else if (angleDiff < -2) {
                BR.setPower(ChangeP);
                BL.setPower(ChangeP);
                FL.setPower(ChangeP + GyroScalePower);
                FR.setPower(ChangeP + GyroScalePower);
            } else {
                BR.setPower(ChangeP);
                BL.setPower(ChangeP);
                FL.setPower(ChangeP);
                FR.setPower(ChangeP);
            }

            if (error < .25 || runtime.seconds() > timeoutS || Math.abs(ChangeP) < .15) {
                stopMotors();
                break;
            }

            this.opMode.telemetry.addData("horizontal:", getEncoderAvg());
            this.opMode.telemetry.addData("YawAngle:", getGyroYaw());
            this.opMode.telemetry.update();
        }
        stopMotors();
    }


    public void gyroInch(double speed, double inches, double timeoutS, int heading) {

        while (this.opMode.opModeIsActive() && !this.opMode.isStopRequested()) {

            double ticks = inches * 32.4;
            double kP = speed / 15;
            heading = -heading;

            //runtime isn't used, this is just a backup call which we don't need
            resetEncoders();
            runtime.reset();

            //if the position is less than the number of inches, than it sets the motors to speed
            while (Math.abs(getEncoderAvg()) <= ticks && this.opMode.opModeIsActive()) {

                double error = (ticks - Math.abs(getEncoderAvg())) / 32.4;
                double ChangeP = error * kP;
                double AngleDiff = GimbleCalc(heading, getGyroYaw());
                double GyroScalePower = 0;
                if (Math.abs(AngleDiff) > 0)
                {
                    GyroScalePower = AngleDiff * .05;
                }


                if (ChangeP > 1)
                    ChangeP = ChangeP / ChangeP;

                //signs could be flipped
                BR.setPower(ChangeP - GyroScalePower);
                FL.setPower(ChangeP + GyroScalePower);
                FR.setPower(ChangeP - GyroScalePower);
                BL.setPower(ChangeP + GyroScalePower);

                this.opMode.telemetry.addData("MotorPowLeft:", ChangeP + GyroScalePower);
                this.opMode.telemetry.addData("MotorPowRight:", ChangeP + GyroScalePower);
                this.opMode.telemetry.addData("heading:", heading);
                this.opMode.telemetry.addData("YawAngle:", getGyroYaw());
                this.opMode.telemetry.addData("angle diff:", AngleDiff);
                this.opMode.telemetry.update();

                if (Math.abs(ChangeP) < .05 || runtime.seconds() >= timeoutS || error < .25) {
                    stopMotors();
                    break;
                }
            }
            break;
        }
        stopMotors();
    }

    public double getTrueDiff(double origAngle) {
        double currAngle = getGyroYaw();
        if (currAngle >= 0 && origAngle >= 0 || currAngle <= 0 && origAngle <= 0)
            return (currAngle - origAngle);
        else if (Math.abs(currAngle - origAngle) <= 180)
            return (currAngle - origAngle);
        else if (currAngle > origAngle)
            return -(360 - (currAngle - origAngle));
        else
            return (360 + (currAngle - origAngle));
    }

    public void turnPD(double angle, double p, double d, double timeout) {

        runtime.reset();
        while (this.opMode.opModeIsActive() && !this.opMode.isStopRequested()) {

            double kP = p / 33;
            double kD = d / .70;

            double currentTime = runtime.milliseconds();
            double pastTime = 0;

            double prevAngleDiff = getTrueDiff(-angle);
            double angleDiff = prevAngleDiff;

            double changePID = 0;

            while (Math.abs(angleDiff) > .95 && runtime.seconds() < timeout && this.opMode.opModeIsActive()) {

                pastTime = currentTime;
                currentTime = runtime.milliseconds();
                double dT = currentTime - pastTime;

                angleDiff = getTrueDiff(-angle);
                changePID = (angleDiff * kP) + ((angleDiff - prevAngleDiff) / dT * kD);

                //signs could be flipped
                if (changePID <= 0) {
                    startMotors(-changePID + .10, changePID - .10, -changePID + .10, changePID - .10);
                } else {
                    startMotors(-changePID - .10, changePID + .10, -changePID - .10, changePID + .10);
                }

                this.opMode.telemetry.addData("P", (angleDiff * kP));
                this.opMode.telemetry.addData("D", ((Math.abs(angleDiff) - Math.abs(prevAngleDiff)) / dT * kD));
                this.opMode.telemetry.addData("angle:", getGyroYaw());
                this.opMode.telemetry.addData("error", angleDiff);
                this.opMode.telemetry.update();

                if (runtime.seconds() > timeout)
                    break;

                prevAngleDiff = angleDiff;
            }
            stopMotors();

            angleDiff = getTrueDiff(-angle);
            if (!(Math.abs(angleDiff) > .95)) {
                break;
            }
        }
    }

    public void updateGyroValues() {
        angles = imu.getAngularOrientation();
    }

    public double getGyroYaw() {
        updateGyroValues();
        return angles.firstAngle;
    }

    public void startMotors(double fleft, double fright, double bleft, double bright) {
        if (!this.opMode.isStopRequested() && this.opMode.opModeIsActive()) {
            BR.setPower(bright);
            FL.setPower(fleft);
            FR.setPower(fright);
            BL.setPower(bleft);
        }
    }

    public void stopMotors() {
        if (!this.opMode.isStopRequested() && this.opMode.opModeIsActive()) {
            BR.setPower(0);
            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
        }
    }

    public double GimbleCalc(double initVal, double curVal) {
        double angleDiff;
        if (initVal - curVal >= 180)
            angleDiff = -1 * ((360 + curVal) - initVal);
        else if (curVal - initVal >= 180)
            angleDiff = (360 + initVal) - curVal;
        else
            angleDiff = initVal - curVal;

        return angleDiff;
    }

    public int getEncoderAvg() {
        int count = 4;
        if ((FR.getCurrentPosition()) == 0) {
            count--;
        }
        if ((FL.getCurrentPosition()) == 0) {
            count--;
        }
        if ((BR.getCurrentPosition()) == 0) {
            count--;
        }
        if ((BL.getCurrentPosition()) == 0) {
            count--;
        }
        count = count == 0 ? 1 : count;

        return (Math.abs(FR.getCurrentPosition()) +  Math.abs(FL.getCurrentPosition()) + Math.abs(BR.getCurrentPosition()) + Math.abs(BL.getCurrentPosition())) / count;
    }
}
