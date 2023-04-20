package OceanCrashPurePursuit.autonomous.waypoints;

import OceanCrashPurePursuit.autonomous.PurePursuitPath;
import OceanCrashPurePursuit.robot.CombinedRobot;

public class Start implements Subroutines.OnceOffSubroutine {

    @Override
    public void runOnce(CombinedRobot robot) {
        robot.actionCache.clear();
        robot.actionCache.add(new DelayedSubroutine(0, Subroutines.DEPOSIT_FOUR_BAR, "FIRSTRUN"));
        robot.actionCache.add(new DelayedSubroutine(500, Subroutines.SWIVEL_OUT, "SWIVELOUT"));
        robot.actionCache.add(new DelayedSubroutine(1200, Subroutines.NEUTRAL_FOUR_BAR, "NEUTRALFOURBAR"));
    }
}
