package OceanCrashOpMode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;
import java.util.List;


@Config
@TeleOp(name = "TeleOp", group = "opMode")
public class OceanCrashTeleOp extends OceanCrashOpMode{


    private ElapsedTime fourbar = new ElapsedTime();
    private ElapsedTime macroTime = new ElapsedTime();
    private ElapsedTime grabTime = new ElapsedTime();
    private ElapsedTime jHeightTime = new ElapsedTime();
    private ElapsedTime grab = new ElapsedTime();
    private ElapsedTime extended = new ElapsedTime();

    private boolean grabbed = false;
    private boolean extend = false;
    private boolean active = false;

    private enum LiftState {
        IDLE,
        RAISE,
        PLACE,
        LOWER,
    }

    private LiftState lift = LiftState.IDLE;
    private String liftState = "IDLE";

    private int jHeight = 3;
    private int liftTargetPos = 0;
    public static int low = 1000, medium = 1570, high = 2320;

    public void loop() {

        // DRIVE
        if (Math.abs(gamepad1.left_stick_x) > .1 || Math.abs(gamepad1.left_stick_y) > .1 || Math.abs(gamepad1.right_stick_x) > .1) {
            drive(gamepad1.right_stick_x, gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_trigger);
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
        telemetry.addData("Stick :: ", gamepad2.left_stick_y);
        telemetry.addData("Encoder :: ", getMotorEncoders());
        telemetry.addData("Red :: ", colorS.red());
        telemetry.addData("Green :: ", colorS.green());
        telemetry.addData("Blue :: ", colorS.blue());
        telemetry.addData("red? :: ", grabRed());
        telemetry.addData("blue? :: ", grabBlue());
        telemetry.addData("liftState :: ", liftState);
        telemetry.addData("jit :: ", jHeight);
        telemetry.addData("exttimer :: ", extended.milliseconds());
        telemetry.addData("extBool :: ", extend);
        telemetry.addData("i should go into idle :: ", !(getLiftPos() > 450));
        telemetry.update();



        // LIFT

        if (gamepad1.dpad_up && jHeight < 3 && jHeightTime.milliseconds() > 200) {
            jHeight++;
            jHeightTime.reset();
        }

        if (gamepad1.dpad_down && jHeight > 0 && jHeightTime.milliseconds() > 200) {
            jHeight--;
            jHeightTime.reset();
        }

        switch (jHeight) {
            case 0:
                liftTargetPos = 100;
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
                if (gamepad2.a && grabTime.milliseconds() > 200) {
                    grabbed = true;
                    grabTime.reset();
                }
                if ((grabRed() || grabBlue()) || grabbed) {
                    if (getLiftPos() > 50)
                        liftReset(.3, 0);
                    else {
                        setLiftPower(0);
                        grab();
                    }
                } else {
                    if (getLiftPos() < 450)
                        setLiftPos(450);
                    else {
                        setLiftPower(0);
                        release();
                    }
                }
                if (gamepad2.right_bumper && !active) {
                    grabbed = false;
                    active = true;
                    lift = LiftState.RAISE;
                }
                break;
            case RAISE:
                if (getLiftPos() < liftTargetPos) {
                    setLiftPos(liftTargetPos);
                } else {
                    extendFourBar();
                    extend = true;
                    if (macroTime.milliseconds() > 400) {
                        macroTime.reset();
                        lift = LiftState.PLACE;
                    }
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
                if (Math.abs(gamepad2.left_stick_y) > .05) {
                    setLiftPower(gamepad2.left_stick_y * 0.2);
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
                if (grabTime.milliseconds() > 100) {
                    grab();
                    retractFourBar();
                }
                if (macroTime.milliseconds() > 200) {
                    if (getLiftPos() > 450) {
                        liftReset(.7, 400);
                    } else {
                        setLiftPower(0);
                        active = false;
                        grabbed = false;
                        release();
                        lift = LiftState.IDLE;
                    }
                }
                liftState = "LOWER";
                break;
            default:
                lift = LiftState.IDLE;
                break;
        }



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


    }
}
