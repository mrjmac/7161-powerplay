package OceanCrashOpMode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import OceanCrashLinearOpMode.Lift;


@Config
@TeleOp(name = "judging", group = "opMode")
public class JudgingTeleOp extends OceanCrashOpMode{


    private final ElapsedTime fourbar = new ElapsedTime();
    private final ElapsedTime drop = new ElapsedTime();

    private boolean grabbed = false;
    private boolean extend = false;
    private boolean goAhead = false;
    private boolean blue = false;
    private boolean doNotReset = false;

    private enum LiftState {
        IDLE,
        PLACE,
        RAISE,
        GRAB,
        RETRACT,
        DONE
    }

    private LiftState lift = LiftState.IDLE;
    private String liftState = "IDLE";

    public void loop() {

        switch (lift) {
            case IDLE:
                setIntake(-.8);
                // keep height at 75
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
                // driver 2 manual movement for lift, automatic grab
                if (grabRed() || blue)
                {
                    doNotReset = true;
                    if (getLiftPos() > 10)
                    {
                        liftReset(.5, 5);
                    }
                    else
                    {
                        setLiftPower(0);
                        grab();
                        fourbar.reset();
                        drop.reset();
                        lift = LiftState.GRAB;
                    }
                }
                break;
            case GRAB:
                setIntake(0);
                if (fourbar.milliseconds() > 250)
                {
                    extendFourBar();
                    lift = LiftState.RAISE;
                }
                liftState = "GRAB";
                break;
            case RAISE:
                swivelIn();
                if (getLiftPos() < 200) {
                    setLiftPos(200);
                } else {
                    setLiftPower(-0.0005);
                    blue = false;
                    extendFourBar();
                    extend = true;
                    drop.reset();
                    lift = LiftState.PLACE;
                }
                liftState = "RAISE";
                break;
            case PLACE:
                setLiftPower(-0.0005);
                if (drop.milliseconds() > 1000 && !goAhead)
                {
                    release();
                    grabbed = false;
                    goAhead = true;

                }
                if (drop.milliseconds() > 2000 && goAhead)
                {
                    drop.reset();
                    grab();
                    grabbed = true;
                    liftState = "RETRACT";
                    lift = LiftState.RETRACT;
                }
                break;
            case RETRACT:
                setLiftPower(0);
                if (drop.milliseconds() > 500)
                {
                    retractFourBar();
                    extend = false;
                    doNotReset = false;
                    liftState = "IDLE";
                    lift = LiftState.IDLE;
                }
            case DONE:
                setLiftPower(0);
                break;
        }

        if (grabBlue())
        {
            blue = true;
        }
        else
        {
            blue = false;
        }

        if (isTouch())
        {
            resetLiftEncoder();
        }

        telemetry.addData("STATE :: ", liftState);
        telemetry.addData("DROP :: ", drop.milliseconds());
        telemetry.addData("GOAHEAD :: ", goAhead);

    }
}
