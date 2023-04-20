package OceanCrashPurePursuit.autonomous.waypoints;

import OceanCrashPurePursuit.robot.CombinedRobot;

public class ReadyToDeposit implements Subroutines.OnceOffSubroutine {

    @Override
    public void runOnce(CombinedRobot robot) {
        robot.actionCache.clear();
        robot.actionCache.add(new DelayedSubroutine(100, Subroutines.DEPOSIT_FOUR_BAR, "DEPOSITFOURBAR"));
        robot.actionCache.add(new DelayedSubroutine(250, Subroutines.RAISE_LIFT, "RAISELIFT"));
    }
}