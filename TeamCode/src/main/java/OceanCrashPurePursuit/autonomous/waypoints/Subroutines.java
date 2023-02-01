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

}
