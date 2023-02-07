package OceanCrashPurePursuit.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import OceanCrashPurePursuit.autonomous.PurePursuitPath;
import OceanCrashPurePursuit.autonomous.odometry.TwoWheelTrackingLocalizer;
import OceanCrashPurePursuit.autonomous.waypoints.DelayedSubroutine;
import OceanCrashPurePursuit.common.math.TimePose;
import OceanCrashPurePursuit.common.util.AxesSigns;
import OceanCrashPurePursuit.common.util.BNO055IMUUtil;
import OceanCrashPurePursuit.common.util.LoadTimer;
import OceanCrashPurePursuit.common.math.Pose;
import OceanCrashPurePursuit.robot.util.MecanumPowers;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

import java.util.Arrays;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

@Config

public class CombinedRobot {

    public Telemetry telemetry;
    private FtcDashboard dashboard;
    public TelemetryPacket packet;

    private Telemetry.Item[] telOdometry;
    private Telemetry.Item[] telEncoders;
    private Telemetry.Item[] telPowers;
    private Telemetry.Item[] telAnalog;

    private Telemetry.Item telDigital;
    private Telemetry.Item telLoopTime;
    private Telemetry.Item telHertz;
    private long lastTelemetryUpdate;

    /* Internal state */
    /* Odometry */
    private double headingOffset;
    public TwoWheelTrackingLocalizer localizer;

    /* Action cache */
    public LinkedList<DelayedSubroutine> actionCache;

    /* Misc. state */
    public double lastHeading;

    private MecanumPowers powers;

    /* Components */
    public BNO055IMU imu;

    public List<DcMotorEx> allMotors;
    public List<Servo> allServos;

    public DcMotorEx BL; // [E1]
    public DcMotorEx BR; // [C0]
    public DcMotorEx FL; // [E0]
    public DcMotorEx FR; // [C1]
    public List<DcMotorEx> chassisMotors;

    public DcMotorEx intakeL; // [E1]
    public DcMotorEx intakeR; // [C0]

    public DcMotorEx liftL; // [E1]
    public DcMotorEx liftR; // [C0]

    public TouchSensor touch;

    public Servo spinL; // [C0]
    public Servo spinR; // [E5]
    public Servo grab; // [C2]
    public Servo swivel;

    /* Uneditable constants */
    public final static double TRACK_WIDTH = 13.83; // in
    public final static double WHEEL_DIAMETER = 1.8898 * 2; // in

    /**
     * Instantiates a <b>real</b> SkystoneHardware object that will try to communicate with the REV
     * hub. Always requires a start position.
     *
     * @param hardwareMap The hardwareMap from which to read our devices
     * @param start The robot's starting location on the field
     */
    public CombinedRobot(HardwareMap hardwareMap, Telemetry telemetry, FtcDashboard dashboard, Pose start) {
        LoadTimer loadTime = new LoadTimer();

        /* Copy dashboard */
        this.dashboard = dashboard;

        /* Drivetrain */
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");

        FR.setDirection(DcMotorEx.Direction.REVERSE);
        BR.setDirection(DcMotorEx.Direction.REVERSE);
        FL.setDirection(DcMotorEx.Direction.FORWARD);
        BL.setDirection(DcMotorEx.Direction.FORWARD);

        FR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        powers = new MecanumPowers(0, 0, 0, 0);
        chassisMotors = Arrays.asList(FL, FR, BL, BR);

        /* Intake */
        intakeL = hardwareMap.get(DcMotorEx.class, "intakeL"); // [E3]
        intakeR = hardwareMap.get(DcMotorEx.class, "intakeR"); // [C3]

        intakeL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeL.setDirection(DcMotorEx.Direction.REVERSE);
        intakeL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakeL.setPower(0);

        intakeR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeR.setDirection(DcMotorEx.Direction.FORWARD);
        intakeR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakeR.setPower(0);

        /* Lift */
        liftL = hardwareMap.get(DcMotorEx.class, "liftL"); // [E2]
        liftR = hardwareMap.get(DcMotorEx.class, "liftR"); // [C2]

        liftL.setDirection(DcMotorSimple.Direction.FORWARD);
        liftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftR.setDirection(DcMotorSimple.Direction.REVERSE);
        liftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        spinL = hardwareMap.servo.get("spinL"); // [C0]
        spinR = hardwareMap.servo.get("spinR"); // [E5]
        grab = hardwareMap.servo.get("grab"); // [E4]
        swivel = hardwareMap.servo.get("swivel");

        touch = hardwareMap.touchSensor.get("touch");

        allMotors = Arrays.asList(FL, BL, FR, BR, intakeL, intakeR, liftL, liftR);
        allServos = Arrays.asList(spinL, spinR, grab, swivel);

        LoadTimer calTime = new LoadTimer();
        initBNO055IMU(hardwareMap);
        calTime.stop();

        TimePose startPose4D = new TimePose(start, System.currentTimeMillis());
        localizer = new TwoWheelTrackingLocalizer(0, 1, startPose4D);
        this.lastHeading = 0;

        /* Action cache */
        actionCache = new LinkedList<>();

        this.telemetry = telemetry;
        initTelemetry();
        logBootTelemetry(hardwareMap, loadTime, calTime);

    }

    public Pose pose() {
        return localizer.pose();
    }

    private void initTelemetry() {
        telemetry.setMsTransmissionInterval(50); // Update at 20 Hz
        telemetry.setAutoClear(false); // Force not to autoclear
        telemetry.setItemSeparator("; ");
        telemetry.setCaptionValueSeparator(" ");
    }

    private void logBootTelemetry(HardwareMap hardwareMap, LoadTimer lT, LoadTimer cT) {
        Telemetry.Log log = telemetry.log();
        log.clear();
        log.setCapacity(6);

        log.add("-- 7161 RC --");

        // Robot information
        List<LynxModule> revHubs = hardwareMap.getAll(LynxModule.class);
        List<DcMotor> motors = hardwareMap.getAll(DcMotor.class);
        List<Servo> servos = hardwareMap.getAll(Servo.class);
        List<DigitalChannel> digital = hardwareMap.getAll(DigitalChannel.class);
        List<AnalogInput> analog = hardwareMap.getAll(AnalogInput.class);
        List<I2cDevice> i2c = hardwareMap.getAll(I2cDevice.class);
        log.add(revHubs.size() + " Hubs; " + motors.size() + " Motors; " + servos.size() +
                " Servos; " + (digital.size() + analog.size() + i2c.size()) + " Sensors");

        lT.stop();

        // Load information
        log.add("Total time " + lT.millis() + " ms; Calibrate time " + cT.millis() + " ms");
        telemetry.update();
        lastTelemetryUpdate = System.nanoTime();
    }

    private void initBNO055IMU(HardwareMap hardwareMap) {
        imu = hardwareMap.get(BNO055IMUImpl.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.loggingEnabled  = false;
        imu.initialize(parameters);
        BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);
        headingOffset = imu.getAngularOrientation().firstAngle;
    }

    public void initBulkReadTelemetry() {
        Telemetry.Line odometryLine = telemetry.addLine();
        telOdometry = new Telemetry.Item[3];
        telOdometry[0] = odometryLine.addData("X", "0");
        telOdometry[1] = odometryLine.addData("Y", "0");
        telOdometry[2] = odometryLine.addData("θ", "0");

        Telemetry.Line encoderLine = telemetry.addLine();
        telEncoders = new Telemetry.Item[4];
        for (int i = 0; i < 4; i++) {
            telEncoders[i] = encoderLine.addData("E" + i, -1);
        }

        Telemetry.Line powersLine = telemetry.addLine();
        telPowers = new Telemetry.Item[4];
        telPowers[0] = powersLine.addData("FL", "0");
        telPowers[1] = powersLine.addData("FR", "0");
        telPowers[2] = powersLine.addData("BL", "0");
        telPowers[3] = powersLine.addData("BR", "0");

        Telemetry.Line analogLine = telemetry.addLine();
        telAnalog = new Telemetry.Item[4];
        for (int i = 0; i < 4; i++) {
            telAnalog[i] = analogLine.addData("A" + i, -1);
        }

        telDigital = telemetry.addLine().addData("DIGITALS", "0 0 0 0 0 0 0 0");

        Telemetry.Line timingLine = telemetry.addLine("LOOP ");
        telHertz = timingLine.addData("Hertz", -1);
        telLoopTime = timingLine.addData("Millis", -1);
    }

    public void performBulkRead() {
        this.lastHeading = imu.getAngularOrientation().firstAngle - headingOffset;
        telOdometry[0].setValue(String.format("%.1f", localizer.x()));
        telOdometry[1].setValue(String.format("%.1f", localizer.y()));
        telOdometry[2].setValue(String.format("%.1f", Math.toDegrees(localizer.h())));

        telPowers[0].setValue(String.format("%.2f", powers.frontLeft));
        telPowers[1].setValue(String.format("%.2f", powers.frontRight));
        telPowers[2].setValue(String.format("%.2f", powers.backLeft));
        telPowers[3].setValue(String.format("%.2f", powers.backRight));

        // Adjust digital inputs
        localizer.update(intakeL.getCurrentPosition(), intakeR.getCurrentPosition(), lastHeading);

        // Run any cached actions
        Iterator<DelayedSubroutine> iterator = actionCache.listIterator();
        long timeMillis = System.currentTimeMillis();
        while(iterator.hasNext()) {
            DelayedSubroutine action = iterator.next();
            if (action.systemActionTime < timeMillis) {
                action.action.runOnce(this);
                iterator.remove();
            }
        }

        double elapsed = ((System.nanoTime() - lastTelemetryUpdate) / 1000000.0);
        telLoopTime.setValue("%.1f", elapsed);
        telHertz.setValue("%.1f", 1000 / elapsed);

        // Finalize telemetry update
        telemetry.update();

        this.packet = new TelemetryPacket();
        packet.put("x", localizer.x());
        packet.put("y", localizer.y());
        packet.put("h", localizer.h());


        packet.fieldOverlay()
                .setFill("blue")
                .fillCircle(localizer.x(), localizer.y(), 3);



        lastTelemetryUpdate = System.nanoTime();
    }

    public void drawDashboardPath(PurePursuitPath path) {
        path.draw(packet.fieldOverlay());
    }

    public void sendDashboardTelemetryPacket() {
        if (dashboard != null) {
            dashboard.sendTelemetryPacket(packet);
        }
    }

    public boolean hasAction(String tag) {
        for (DelayedSubroutine s : actionCache) {
            if (tag.equals(s.tag)) {
                return true;
            }
        }
        return false;
    }

    public void setPowers(MecanumPowers powers) {
        this.powers = powers;
        FL.setPower(powers.frontLeft);
        FR.setPower(powers.frontRight);
        BL.setPower(powers.backLeft);
        BR.setPower(powers.backRight);
    }

}
