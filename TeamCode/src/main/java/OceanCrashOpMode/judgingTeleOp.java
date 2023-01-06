package OceanCrashOpMode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import OceanCrashLinearOpMode.Lift;


@Config
@TeleOp(name = "judging", group = "opMode")
public class judgingTeleOp extends OceanCrashOpMode{


    private final ElapsedTime fourbar = new ElapsedTime();
    private final ElapsedTime drop = new ElapsedTime();

    private boolean grabbed = false;
    private boolean bypass = false;
    private boolean extend = false;
    private boolean goAhead = false;
    private boolean blue = false;
    private boolean doNotReset = false;

    private enum LiftState {
        IDLE,
        PLACE,
        RETRACT,
    }

    private LiftState lift = LiftState.IDLE;
    private String liftState = "IDLE";

    public void loop() {

        switch (lift) {
            case IDLE:
                // keep height at 75
                if (!doNotReset)
                {
                    if (getLiftPos() < 65)
                    {
                        setLiftPosLittle(80);
                    }
                    else
                    {
                        setLiftPower(-0.0005);
                    }
                }
                // driver 2 manual movement for lift, automatic grab
                if (grabRed() || blue || bypass)
                {
                    doNotReset = true;
                    if (getLiftPos() > 5)
                    {
                        liftReset(.6, 0);
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
                        if (fourbar.milliseconds() > 500)
                        {
                            extendFourBar();
                            bypass = false;
                            drop.reset();
                            liftState = "PLACE";
                            lift = LiftState.PLACE;
                        }
                    }
                }
                break;
            case PLACE:
                setLiftPower(0);
                if (drop.milliseconds() > 500 && !goAhead)
                {
                    release();
                    grabbed = false;
                    goAhead = true;

                }
                if (drop.milliseconds() > 1500 && goAhead)
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
