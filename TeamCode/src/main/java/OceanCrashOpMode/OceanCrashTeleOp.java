package OceanCrashOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import OceanCrashLinearOpMode.Lift;

@TeleOp(name = "TeleOp", group = "opMode")
public class OceanCrashTeleOp extends OceanCrashOpMode{


    private ElapsedTime fourbar = new ElapsedTime();
    private boolean grabbed = false, extend = false;
    private ElapsedTime grab = new ElapsedTime(), extended = new ElapsedTime();

    public void loop() {

        // DRIVE
        if (Math.abs(gamepad1.left_stick_x) > .1 || Math.abs(gamepad1.left_stick_y) > .1 || Math.abs(gamepad1.right_stick_x) > .1)
        {
            drive(gamepad1.right_stick_x, gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_trigger);
        }
        else
        {
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


        if (Math.abs(gamepad2.left_stick_y) > .05)
        {
            setLiftPower(gamepad2.left_stick_y * 0.7);
        }
        if (gamepad1.a)
        {
            setLiftPower(0);
        }
        /*
        else if (getLiftPos() > 1000)
        {
            setLiftPower(-.0005);
        }
        else
        {
            setLiftPower(0);
        }

        if (gamepad2.a && grab.milliseconds() > 250 && !grabbed)
        {
            grab.reset();
            grabbed = true;
            grab();
        }
        */


        if (gamepad2.a && grab.milliseconds() > 250 && grabbed)
        {
            grab.reset();
            grabbed = false;
            release();
        }

        if (gamepad2.y && extended.milliseconds() > 250 && extend)
        {
            if (!grabbed) {
                grab();
                grabbed = true;
            }
            extended.reset();
            extend = false;
            retractFourBar();
        }

        if (gamepad2.y && extended.milliseconds() > 250 && !extend)
        {
            if (!grabbed) {
                grab();
                grabbed = true;
            }
            extended.reset();
            extend = true;
            extendFourBar();
        }


        telemetry.addData("Lift Pos :: ", getLiftPos());
        telemetry.addData("Stick :: ", gamepad2.left_stick_y);
        telemetry.addData("Encoder :: ", getMotorEncoders());
        telemetry.addData("Red :: ", colorS.red());
        telemetry.addData("Green :: ",colorS.green());
        telemetry.addData("Blue :: ", colorS.blue());
        telemetry.addData("red? :: ", grabRed());
        telemetry.addData("blue? :: ", grabBlue());
        telemetry.addData("liftState :: ", liftState);
        telemetry.addData("jit :: ", jHeight);
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
                liftTargetPos = 1300;
                break;
            case 2:
                liftTargetPos = 2000;
                break;
            case 3:
                liftTargetPos = 2700;
        }

        switch (lift) {
            case IDLE:
                if (grabRed() || grabBlue()) {
                    if (getLiftPos() > 50)
                        liftReset(.1, 0);
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
                if (gamepad2.left_bumper && !active) {
                    active = true;
                    lift = LiftState.RAISE;
                }

               /* if (gamepad2.right_bumper && !active)
                    lift = LiftState.BEACON;
                liftState = "IDLE";

                */
                break;
           /* case BEACON:
                extendFourBar();
                if (gamepad2.a && grabTime.milliseconds() > 200) {
                    if (!grabbed)
                        release();
                    else
                        grab();
                    grabTime.reset();
                }
                if (gamepad2.left_bumper && macroTime.milliseconds() > 200)
                {
                    active = true;
                    lift = LiftState.RAISE;
                }
                liftState = "BEACON";
                break;
            */
            case RAISE:
                if (getLiftPos() < liftTargetPos) {
                    setLiftPos(liftTargetPos);
                } else {
                    if (jHeight == 0)
                        setLiftPower(0);
                    else if (jHeight == 1)
                        setLiftPower(-.00045);
                    else
                        setLiftPower(-.0005);
                    if (!grabbed)
                        grab();
                    extendFourBar();
                    if (macroTime.milliseconds() > 750) {
                        macroTime.reset();
                        lift = LiftState.PLACE;
                    }
                }
                liftState = "RAISE";
                break;
            case PLACE:
                //setLiftPower(.12); //stallpower, calculate using torque and mg
                if (gamepad2.a && grabTime.milliseconds() > 200) {
                    release(); //need to test timing, will prob have to modify delay using macroTime
                    grabTime.reset();
                    lift = LiftState.LOWER;
                }
                liftState = "PLACE";
                break;
            case LOWER:
                if (grabTime.milliseconds() > 200) {
                    grab();
                    retractFourBar();
                }
                if (getLiftPos() > 450)
                    liftReset(.5, 400);
                else
                {
                    setLiftPower(0);
                    active = false;
                    lift = LiftState.IDLE;
                }
                liftState = "LOWER";
                break;
            default:
                lift = LiftState.IDLE;
                break;
        }
/*
        // FOUR BAR
        if (gamepad2.left_trigger > .1)
        {
            retractFourBar();
        }
        if (gamepad2.right_trigger > .1)
        {
            extendFourBar();
        }
        if (gamepad2.a)
        {
            grab();
        }
        if (gamepad2.b)
        {
            release();
        }
        */



        telemetry.update();
    }
}
