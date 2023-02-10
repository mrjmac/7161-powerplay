package OceanCrashPurePursuit.autonomous.waypoints;

import com.acmerobotics.dashboard.config.Config;

import OceanCrashPurePursuit.autonomous.PurePursuitPath;
import OceanCrashPurePursuit.robot.CombinedRobot;

@Config
public class Subroutines {

    public interface Subroutine {}

    public interface OnceOffSubroutine extends Subroutine {
        void runOnce(CombinedRobot robot);
    }

    public interface MetaSubroutine extends Subroutine {
        void runOnce(PurePursuitPath path, CombinedRobot robot);
    }

    public interface RepeatedSubroutine extends Subroutine {
        boolean runLoop(CombinedRobot robot, PurePursuitPath path); // Returns whether we should advance
    }

    // Once a position is reached, if that position has an ArrivalInterruptSubroutine, we advance to
    // the next waypoint (so the waypoint with the ArrivalInterruptSubroutine is our current waypoint)
    // and we call runCycle every tick until it eventually returns true
    public interface ArrivalInterruptSubroutine extends Subroutine {
        boolean runCycle(CombinedRobot robot); // Returns whether it's complete
    }

    public static final OnceOffSubroutine OPEN_CLAW = (robot) -> { robot.grab(); };

    public static final OnceOffSubroutine CLOSE_CLAW = (robot) -> { robot.release(); };

    public static final OnceOffSubroutine GRAB_FOUR_BAR = (robot) -> { robot.extendFourBar();};

    public static final OnceOffSubroutine DEPOSIT_FOUR_BAR = (robot) -> { robot.trueExtendFourBar(); };

    public static final OnceOffSubroutine RAISE_LIFT = (robot) -> { robot.setSlideTarget(850); };

    public static final OnceOffSubroutine LOWER_LIFT = (robot) -> { robot.setSlideTarget(0); };


    public static final OnceOffSubroutine RETRACT_FOUR_BAR = (robot) -> { robot.retractFourBar();};

    public static final OnceOffSubroutine SWIVEL_OUT = (robot) -> { robot.swivelOut();};

    public static final OnceOffSubroutine SWIVEL_IN = (robot) -> { robot.swivelIn();};

    public static final OnceOffSubroutine SWIVEL_START_LEFT = (robot) -> { robot.swivelStartLeft();};

    public static final OnceOffSubroutine SWIVEL_START_RIGHT = (robot) -> { robot.swivelStartRight();};









}
