package OceanCrashPurePursuit.autonomous.waypoints;

import OceanCrashPurePursuit.robot.CombinedRobot;

public class ReadyToGrab implements Subroutines.OnceOffSubroutine {

    @Override
    public void runOnce(CombinedRobot robot) {
        robot.actionCache.clear();
        robot.actionCache.add(new DelayedSubroutine(0, Subroutines.GRAB_FOUR_BAR, "GRABFOURBAR"));
        robot.actionCache.add(new DelayedSubroutine(0, Subroutines.POST_GRAB_LIFT, "RAISELIFT"));
        robot.actionCache.add(new DelayedSubroutine(200, Subroutines.OPEN_CLAW, "OPENCLAW"));
    }
}