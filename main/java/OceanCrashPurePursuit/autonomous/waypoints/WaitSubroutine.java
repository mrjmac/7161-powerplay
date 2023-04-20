package OceanCrashPurePursuit.autonomous.waypoints;

import com.qualcomm.robotcore.util.ElapsedTime;
import OceanCrashPurePursuit.robot.CombinedRobot;

public class WaitSubroutine implements Subroutines.ArrivalInterruptSubroutine {
    double waitMS;
    ElapsedTime timer;

    public WaitSubroutine(double waitMS) {
        this.waitMS = waitMS;
        this.timer = null;
    }

    @Override
    public boolean runCycle(CombinedRobot robot) {
        if (timer == null) {
            timer = new ElapsedTime();
        }

        return timer.milliseconds() > waitMS;
    }
}
