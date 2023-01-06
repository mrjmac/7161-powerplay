package OceanCrashOpMode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
@TeleOp(name = "TeleOp", group = "opMode")
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
                    if (getLiftPos() < 75)
                    {
                        setLiftPosLittle(75);
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
                        if (fourbar.milliseconds() > 300)
                        {
                            extendFourBar();
                            bypass = false;
                            drop.reset();
                            liftState = "PLACE";

                        }
                    }
                }
                break;
            case PLACE:
                if (drop.milliseconds() > 1000 && !goAhead)
                {
                    release();
                    grabbed = false;
                    goAhead = true;

                }
                if (drop.milliseconds() > 1500 && goAhead)
                {
                    grab();
                    grabbed = true;
                    drop.reset();
                    liftState = "RETRACT";
                }
                break;
            case RETRACT:
                if (drop.milliseconds() > 500)
                {
                    retractFourBar();
                    extend = false;
                    liftState = "IDLE";
                }
        }

        if (grabBlue())
        {
            blue = true;
        }

        if (isTouch())
        {
            resetLiftEncoder();
        }

    }
}
