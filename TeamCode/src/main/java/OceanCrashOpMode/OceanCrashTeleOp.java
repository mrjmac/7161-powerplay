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
    private final ElapsedTime liftTime = new ElapsedTime();

    private boolean grabbed = false;
    private boolean bypass = false;
    private boolean extend = false;
    private boolean active = false;
    private boolean blue = false;
    private boolean manualGrab = false;
    private boolean swivelTrue = false;
    private boolean doNotReset = false;
    private boolean toggle2 = false;
    private boolean firstLoop = true;

    public static double wait = 200;
    private boolean grabd1 = false;
    private boolean mG = false;
    private boolean liftEdited = false;
    private boolean dtMovement = false;


    private enum LiftState {
        IDLE,
        GRAB,
        RAISE,
        PLACE,
        STACK,
        LOWER,
        DEAD,
    }

    private LiftState lift = LiftState.IDLE;
    private String liftState = "IDLE";

    private int jHeight = 1;
    private int liftTargetPos = 0;
    public static int low = 225, medium = 450, high = 800;


    public void loop() {
        if (firstLoop) {
            liftTime.reset();
            firstLoop = false;
        }

        // DRIVE
        if (Math.abs(gamepad1.left_stick_x) > .1 || Math.abs(gamepad1.left_stick_y) > .1 || Math.abs(gamepad1.right_stick_x) > .1) {
            drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_trigger);
            dtMovement = true;
        } else {
            stopMotors();
        }

        // INTAKE
        if (gamepad1.right_bumper && gamepad1.left_bumper)
            setIntake(1);
        else if (gamepad1.right_bumper)
            setIntake(-1);
        else if (gamepad1.left_bumper)
            setIntake(.25);
        else
            setIntake(0);

        telemetry.addData("Lift Pos :: ", getLiftPos());
        telemetry.addData("lift pow :: ", getPower());
        telemetry.addData("should i not be reseting :: ", doNotReset);
        telemetry.addData("bypass :: ", bypass);
        telemetry.addData("voltage :: ", getVoltage());
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
            liftEdited = false;
        }

        if (gamepad2.dpad_down && jHeightTime.milliseconds() > 125) {
            jHeight = 1;
            jHeightTime.reset();
            liftEdited = false;
        }

        if (gamepad2.dpad_right && jHeightTime.milliseconds() > 125) {
            jHeight = 2;
            jHeightTime.reset();
            liftEdited = false;
        }

        if (gamepad2.dpad_left && jHeightTime.milliseconds() > 125) {
            jHeight = 0;
            jHeightTime.reset();
            liftEdited = false;
        }

        switch (jHeight) {
            case 0:
                liftTargetPos = 30;
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
                updateLiftLength(liftTime.milliseconds());
                if (gamepad2.a)
                {
                    manualGrab = true;
                }
                swivelIn();
                //retractFourBar();
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
                // keep height at 35
                if (!doNotReset)
                {
                    setSlideTarget(50);
                    /*if (getLiftPos() > 60)
                    {
                        setSlideTarget(60);
                    }
                    else if(getLiftPos() < 30)
                    {
                        setSlideTarget(50);
                    }
                    else
                    {
                        setLiftPower(-0.0005);
                    }
                     */
                }
                // driver 2 manual movement for lift, automatic grab
                if ((grabRed() || blue || manualGrab) && reset.milliseconds() > 250)
                {
                    doNotReset = true;
                    if (getLiftPos() > 2 && !manualGrab)
                    {
                        setSlideTarget(0);
                    }
                    else
                    {
                        //setLiftPower(0);
                        grab();
                        fourbar.reset();
                        grabbed = true;
                        doNotReset = false;
                        manualGrab = false;
                        lift = LiftState.GRAB;
                    }
                }
                liftState = "IDLE";
                break;
            case GRAB:
                updateLiftLength(liftTime.milliseconds());
                if (fourbar.milliseconds() > 250)
                {
                    grabFourBar();
                }
                if (gamepad2.right_bumper && !active)
                {
                    active = true;
                    blue = false;
                    lift = LiftState.RAISE;
                }
                if (gamepad1.x)
                {
                    retractFourBar();
                    release();
                    grabbed = false;
                    reset.reset();
                    lift = LiftState.IDLE;
                }
                liftState = "GRAB";
                break;
            case RAISE:
                updateLiftLength(liftTime.milliseconds());
                swivelIn();
                if (Math.abs(getLiftPos() - liftTargetPos) > 2) {
                    //if (jHeight != 0)
                    //{
                        setSlideTarget(liftTargetPos);
                    /*}
                    else
                    {
                        setLiftPosLittle(liftTargetPos);
                    }*/
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
                if (gamepad2.x && swivel.milliseconds() > 250 && grabbed)
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
                    setLiftPower(gamepad2.left_stick_y * 0.20);
                    liftEdited = true;
                } else if (!liftEdited) {
                    setSlideTarget(liftTargetPos);
                    if (jHeight != 0) {
                        extendFourBar();
                    } else {
                        trueExtendFourBar();
                    }
                    updateLiftLength(liftTime.milliseconds());
                    /*if (getLiftPos() > 100)
                        setLiftPower(-.0005);
                    else if (jHeight == 0)
                        setLiftPower(0);
                    else if (jHeight == 1)
                        setLiftPower(-0.0005);
                    else
                        setLiftPower(-0.0005);*/
                } else {
                    if (getLiftPos() > 100)
                        setLiftPower(-.0005);
                    else if (jHeight == 0)
                        setLiftPower(0);
                    else if (jHeight == 1)
                        setLiftPower(-0.0005);
                    else
                        setLiftPower(-0.0005);
                }
                if ((gamepad2.a || gamepad1.a) && grabTime.milliseconds() > 200) {
                    //setLiftPower(-0.0005);
                    release(); //need to test timing, will prob have to modify delay using macroTime
                    dtMovement = false;
                    grabTime.reset();
                    lift = LiftState.LOWER;
                    macroTime.reset();
                }
                if (gamepad2.b && grabTime.milliseconds() > 200) {
                    if (grabbed) {
                        release();
                        grabbed = false;
                    } else {
                        grab();
                        grabbed = true;
                    }
                    grabTime.reset();
                }
                liftState = "PLACE";
                break;
            case LOWER:
                if (!dtMovement) {
                    grabTime.reset();
                    if (jHeight == 0)
                        dtMovement = true;
                } else {
                    swivelIn();
                    //}
                    if (grabTime.milliseconds() > 500) {
                        grab();
                        updateLiftLength(liftTime.milliseconds());
                        retractFourBar();
                    }
                    if ((macroTime.milliseconds() > 750 && jHeight > 0) || (jHeight == 0 && macroTime.milliseconds() > 1000)) {
                        if (getLiftPos() > 25) {
                            setSlideTarget(25);
                        } else {
                            //setLiftPower(0);
                            active = false;
                            grabbed = false;
                            extend = false;
                            release();
                            lift = LiftState.IDLE;
                        }
                    }
                }
                liftState = "LOWER";
                break;
            case DEAD:
                updateLiftLength(liftTime.milliseconds());
                if (getLiftPos() > 50) {
                    setSlideTarget(50);
                } else {
                    //setLiftPower(0);
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
