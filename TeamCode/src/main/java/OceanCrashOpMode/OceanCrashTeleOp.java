package OceanCrashOpMode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.util.Arrays;
import java.util.List;

import OceanCrashLinearOpMode.Right.Right;
import OceanCrashRoadrunner.drive.SampleMecanumDrive;


@Config
@TeleOp(name = "TeleOp", group = "opMode")
public class OceanCrashTeleOp extends OceanCrashOpMode{


    private final ElapsedTime fourbar = new ElapsedTime();
    private final ElapsedTime macroTime = new ElapsedTime();
    private final ElapsedTime grabTime = new ElapsedTime();
    private final ElapsedTime jHeightTime = new ElapsedTime();
    private final ElapsedTime grab = new ElapsedTime();
    private final ElapsedTime extended = new ElapsedTime();
    private final ElapsedTime swivel = new ElapsedTime();
    private final ElapsedTime reset = new ElapsedTime();

    private boolean grabbed = false;
    private boolean bypass = false;
    private boolean extend = false;
    private boolean active = false;
    private boolean blue = false;
    private boolean resetbo = false;
    private boolean swivelTrue = false;
    private boolean doNotReset = false;
    private boolean toggle2 = false;

    public static double wait = 200;
    private boolean grabd1 = false;
    private boolean mG = false;


    private enum LiftState {
        IDLE,
        RAISE,
        PLACE,
        LOWER,
        DEAD,
    }

    private LiftState lift = LiftState.IDLE;
    private String liftState = "IDLE";

    private int jHeight = 1;
    private int liftTargetPos = 0;
    public static int low = 65, medium = 300, high = 600;


    public void loop() {

        // DRIVE
        if (Math.abs(gamepad1.left_stick_x) > .1 || Math.abs(gamepad1.left_stick_y) > .1 || Math.abs(gamepad1.right_stick_x) > .1) {
            drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_trigger);
        } else {
            stopMotors();
        }

        // INTAKE
        if (gamepad1.right_bumper && gamepad1.left_bumper)
            setIntake(.5);
        else if (gamepad1.right_bumper)
            setIntake(-.9);
        else if (gamepad1.left_bumper)
            setIntake(-.5);
        else
            setIntake(0);

        telemetry.addData("Lift Pos :: ", getLiftPos());
        telemetry.addData("lift pow :: ", getPower());
        telemetry.addData("should i not be reseting :: ", doNotReset);
        telemetry.addData("bypass :: ", bypass);
        //telemetry.addData("Stick :: ", gamepad2.left_stick_y);
        //telemetry.addData("Encoder :: ", getMotorEncoders());
        //telemetry.addData("Red :: ", colorS.red());
        //telemetry.addData("Green :: ", colorS.green());
        //telemetry.addData("Blue :: ", colorS.blue());
        telemetry.addData("red? :: ", grabRed());
        telemetry.addData("blue? :: ", grabBlue());
        telemetry.addData("liftState :: ", liftState);
        telemetry.addData("jit :: ", jHeight);
        telemetry.addData("swivel :: ", swivelTrue);
        telemetry.addData("grab :: ", grabbed);
        //telemetry.addData("extended time :: ", extended.milliseconds());
        telemetry.update();



        // LIFT

        //dpad up high, dpad down low, dpad left ground, dpad right mid
        if (gamepad2.dpad_up && jHeightTime.milliseconds() > 125) {
            jHeight = 3;
            jHeightTime.reset();
        }

        if (gamepad2.dpad_down && jHeightTime.milliseconds() > 125) {
            jHeight = 1;
            jHeightTime.reset();
        }

        if (gamepad2.dpad_right && jHeightTime.milliseconds() > 125) {
            jHeight = 2;
            jHeightTime.reset();
        }

        if (gamepad2.dpad_left && jHeightTime.milliseconds() > 125) {
            jHeight = 0;
            jHeightTime.reset();
        }

        switch (jHeight) {
            case 0:
                liftTargetPos = 10;
                break;
            case 1:
                liftTargetPos = low;
                break;
            case 2:
                liftTargetPos = medium;
                break;
            case 3:
                liftTargetPos = high;
        }


        switch (lift) {
            case IDLE:
                if (gamepad2.a)
                    mG = true;
                swivelIn();
                // change state to raise for driver 2
                if (gamepad2.right_bumper && !active) {
                    grabbed = false;
                    doNotReset = false;
                    active = true;
                    bypass = false;
                    blue = false;
                    mG = false;
                    lift = LiftState.RAISE;
                }
                // manual grab for driver 1
                if (gamepad1.a && grab.milliseconds() > 250 && !grabbed)
                {
                    grab.reset();
                    grabbed = true;
                    grab();
                }
                if (gamepad1.a && grab.milliseconds() > 250 && grabbed)
                {
                    grab.reset();
                    grabbed = false;
                    release();
                }
                //manual reset for driver 2
                if (gamepad2.x && reset.milliseconds() > 250)
                {
                    retractFourBar();
                    release();
                    grabbed = false;
                    doNotReset = false;
                    bypass = false;
                    blue = false;
                    mG = false;
                }
                // keep height at 35
                if (!doNotReset)
                {
                    if (getLiftPos() > 60)
                    {
                        liftReset(.6, 60);
                    }
                    else if(getLiftPos() < 30)
                    {
                        setLiftPosLittle(50);
                    }
                    else
                    {
                        setLiftPower(-0.0005);
                    }
                }
                // de jam for driver 2 with moving four bar back and forth
                if (gamepad2.y && extended.milliseconds() > 250 && !extend)
                {
                    extended.reset();
                    extend = true;
                    extendFourBar();
                }
                if (gamepad2.y && extended.milliseconds() > 250 && extend)
                {
                    extended.reset();
                    extend = false;
                    retractFourBar();
                }
                // driver 2 manual movement for lift, automatic grab
                if (grabRed() || blue || (mG) || bypass)
                {
                    doNotReset = true;
                    if (getLiftPos() > 20)
                    {
                        liftReset(.6, 5);
                    }
                    else
                    {
                        setLiftPower(0);
                        if (!bypass)
                        {
                            fourbar.reset();
                            bypass = true;
                        }
                        grab();
                        if (fourbar.milliseconds() > 300)
                        {
                            grabFourBar();
                            bypass = false;
                        }
                    }
                }
                break;
            case RAISE:
                swivelIn();
                if (getLiftPos() < liftTargetPos) {
                    setLiftPos(liftTargetPos);
                } else {
                    blue = false;
                    if (jHeight != 0)
                    {
                        extendFourBar();
                    }
                    else
                    {
                        trueExtendFourBar();
                    }
                    extend = true;
                    lift = LiftState.PLACE;
                }
                liftState = "RAISE";
                break;
            case PLACE:
                if (gamepad2.y && extended.milliseconds() > 250) {
                    if (!grabbed) {
                        grab();
                        grabbed = true;
                    }
                    if (extend) {
                        extend = false;
                        retractFourBar();
                    } else {
                        extend = true;
                        extendFourBar();
                    }
                    extended.reset();
                }
                if (gamepad2.x && swivel.milliseconds() > 250)
                {
                    if (swivelTrue) {
                        swivelTrue = false;
                        swivelIn();
                    } else {
                        swivelTrue = true;
                        swivelOut();
                    }
                    swivel.reset();
                }
                if (Math.abs(gamepad2.left_stick_y) > .05) {
                    setLiftPower(gamepad2.left_stick_y * 0.1);
                } else {
                    if (jHeight == 0)
                        setLiftPower(0);
                    else if (jHeight == 1)
                        setLiftPower(-0.0005);
                    else
                        setLiftPower(-0.0005);
                }
                if (gamepad2.a && grabTime.milliseconds() > 200) {
                    release(); //need to test timing, will prob have to modify delay using macroTime
                    grabTime.reset();
                    lift = LiftState.LOWER;
                    macroTime.reset();
                }
                liftState = "PLACE";
                break;
            case LOWER:
                swivelIn();
                if (grabTime.milliseconds() > 500) {
                    grab();
                    retractFourBar();
                }
                if ((macroTime.milliseconds() > 500 && jHeight > 0) || (jHeight == 0 && macroTime.milliseconds() > 1000)) {
                    if (getLiftPos() > 25) {
                        liftReset(.4, 25);
                    } else {
                        setLiftPower(0);
                        active = false;
                        grabbed = false;
                        extend = false;
                        release();
                        lift = LiftState.IDLE;
                    }
                }
                liftState = "LOWER";
                break;
            case DEAD:
                if (getLiftPos() > 50) {
                    liftReset(.6, 50);
                } else {
                    setLiftPower(0);
                    active = false;
                    lift = LiftState.IDLE;
                }
                break;
            default:
                lift = LiftState.IDLE;
                break;
        }

        if (grabBlue())
        {
            blue = true;
        }

        if (isTouch())
        {
            resetLiftEncoder();
        }

        if (Math.abs(getLiftL() - getLiftR()) > 125)
            lift = LiftState.DEAD;



        // MANUAL LIFT CODE FOR TESTING
        /*
        if (Math.abs(gamepad2.left_stick_y) > .05) {
            setLiftPower(gamepad2.left_stick_y);
        } else {
            setLiftPower(0);
        }


        if (gamepad2.a && grab.milliseconds() > 250 && !grabbed)
        {
            grab.reset();
            grabbed = true;
            grab();
        }



        if (gamepad2.a && grab.milliseconds() > 250 && grabbed)
        {
            grab.reset();
            grabbed = false;
            release();
        }
        */
        /*
        if (gamepad2.a && grab.milliseconds() > 250 && !grabbed)
        {
            grab.reset();
            grabbed = true;
            grab();
        }



        if (gamepad2.a && grab.milliseconds() > 250 && grabbed)
        {
            grab.reset();
            grabbed = false;
            release();
        }

        if ((gamepad2.y && extended.milliseconds() > 250) || bypass) {
            if (!bypass)
            {
                extended.reset();
                bypass = true;
            }
            if (extend) {
                swivelIn();
                grab();
                if (extended.milliseconds() > wait) {
                    retractFourBar();
                    bypass = false;
                    extended.reset();
                    extend = false;
                }
            } else {
                extendFourBar();
                if (extended.milliseconds() > wait + 550) {
                    swivelOut();
                    bypass = false;
                    extended.reset();
                    extend = true;
                }
            }
        }

        if (gamepad2.x && swivel.milliseconds() > 250)
        {
            if (swivelTrue) {
                swivelTrue = false;
                swivelIn();
            } else {
                swivelTrue = true;
                swivelOut();
            }
            swivel.reset();
        }
         */


    }
}
