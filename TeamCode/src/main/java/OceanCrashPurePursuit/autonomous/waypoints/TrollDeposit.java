package OceanCrashPurePursuit.autonomous.waypoints;

import com.qualcomm.robotcore.util.ElapsedTime;

import OceanCrashPurePursuit.common.math.MathUtil;
import OceanCrashPurePursuit.common.math.Pose;
import OceanCrashPurePursuit.robot.CombinedRobot;
import OceanCrashPurePursuit.robot.util.MecanumPowers;

public class TrollDeposit implements Subroutines.ArrivalInterruptSubroutine {

    public ElapsedTime time;
    public static double TARGET_HEADING = Math.toRadians(225);
    public static double REDUCTION_DIST = Math.PI/3;

    public TrollDeposit() {
        time = null;
    }

    @Override
    public boolean runCycle(CombinedRobot robot) {
        // If we don't have a "no check" action
        if (time == null) {

            time = new ElapsedTime();
            robot.actionCache.clear(); // Remove everything from action cache
            robot.setSlideTarget(850);

            robot.actionCache.add(new DelayedSubroutine(1000, Subroutines.OPEN_CLAW, "OPENCLAW"));
            robot.actionCache.add(new DelayedSubroutine(1500, Subroutines.NEUTRAL_FOUR_BAR, "CONE1DEPOSITEND"));
            robot.actionCache.add(new DelayedSubroutine(1500, Subroutines.LOWER_LIFT, "LOWER_LIFT"));

        }
        if (!robot.hasAction("LOWER_LIFT") && time.milliseconds() > 1500)
        {
            return true;
        }
        else
        {
            double currentHeading = robot.pose().heading;
            double angleToTarget = -MathUtil.angleWrap(TARGET_HEADING - currentHeading);
            Pose poseTurnPower = new Pose(0, 0, angleToTarget / REDUCTION_DIST);
            robot.setPowers(new MecanumPowers(poseTurnPower));

            return false;
        }
    }

}
