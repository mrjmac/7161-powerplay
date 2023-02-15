package OceanCrashPurePursuit.autonomous.waypoints;

import com.qualcomm.robotcore.util.ElapsedTime;

import OceanCrashPurePursuit.common.math.MathUtil;
import OceanCrashPurePursuit.common.math.Pose;
import OceanCrashPurePursuit.robot.CombinedRobot;
import OceanCrashPurePursuit.robot.util.MecanumPowers;

public class Grab implements Subroutines.ArrivalInterruptSubroutine {

    public static double TARGET_HEADING = -Math.toRadians(45);
    public static double REDUCTION_DIST = Math.PI/3;
    public double slideTarget;
    public static ElapsedTime time;

    public Grab(double slideTarget) {
        this.slideTarget = slideTarget;
        time = null;
    }

    @Override
    public boolean runCycle(CombinedRobot robot) {
        // If we don't have a "no check" action
        if (time == null) {

            time = new ElapsedTime();
            robot.actionCache.clear(); // Remove everything from action cache

            robot.setSlideTarget(slideTarget);

            robot.actionCache.add(new DelayedSubroutine(250, Subroutines.CLOSE_CLAW, "CLOSECLAW"));
            robot.actionCache.add(new DelayedSubroutine(600, Subroutines.POST_GRAB_LIFT, "RAISE_LIFT"));
            robot.actionCache.add(new DelayedSubroutine(900, Subroutines.NEUTRAL_FOUR_BAR, "NEUTRALFOURBAR"));



        }
        // If we don't have a deposit end action, we're done!
        if (!robot.hasAction("NEUTRALFOURBAR") && time.milliseconds() > 1500) {
            return true;
        } else {
            /*
            // Might as well be finishing adjusting our heading while we're here
            double currentHeading = robot.pose().heading;
            double angleToTarget = MathUtil.angleWrap(TARGET_HEADING - currentHeading);
            Pose poseTurnPower = new Pose(0, 0, angleToTarget / REDUCTION_DIST);
            robot.setPowers(new MecanumPowers(poseTurnPower));

             */
            return false;
        }
    }

}