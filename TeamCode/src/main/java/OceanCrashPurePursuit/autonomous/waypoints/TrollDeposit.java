package OceanCrashPurePursuit.autonomous.waypoints;

import com.qualcomm.robotcore.util.ElapsedTime;

import OceanCrashPurePursuit.common.math.MathUtil;
import OceanCrashPurePursuit.common.math.Pose;
import OceanCrashPurePursuit.robot.CombinedRobot;
import OceanCrashPurePursuit.robot.util.MecanumPowers;

public class TrollDeposit implements Subroutines.ArrivalInterruptSubroutine {

    public static double TARGET_HEADING = -Math.toRadians(135);
    public static double REDUCTION_DIST = Math.PI/3;
    public static ElapsedTime time;

    public TrollDeposit() {
        time = null;
    }

    @Override
    public boolean runCycle(CombinedRobot robot) {
        // If we don't have a "no check" action
        if (time == null) {

            time = new ElapsedTime();
            robot.actionCache.clear(); // Remove everything from action cache

            robot.setSlideTarget(875);

            //robot.actionCache.add(new DelayedSubroutine(0, Subroutines.DEPOSIT_FOUR_BAR, "FIRSTRUN"));
            robot.actionCache.add(new DelayedSubroutine(400, Subroutines.OPEN_CLAW, "OPENCLAW"));
            robot.actionCache.add(new DelayedSubroutine(1000, Subroutines.NEUTRAL_FOUR_BAR, "CONE1DEPOSITEND"));
            robot.actionCache.add(new DelayedSubroutine(1200, Subroutines.LOWER_LIFT, "LOWER_LIFT"));

        }
        // If we don't have a deposit end action, we're done!
        if (0 == 1) {
            return true;
        } else {
            // Might as well be finishing adjusting our heading while we're her
            double currentHeading = robot.pose().heading;
            double angleToTarget = MathUtil.angleWrap(TARGET_HEADING - currentHeading);
            Pose poseTurnPower = new Pose(0, 0, angleToTarget / REDUCTION_DIST);
            robot.setPowers(new MecanumPowers(poseTurnPower));


            return false;
        }
    }

}
