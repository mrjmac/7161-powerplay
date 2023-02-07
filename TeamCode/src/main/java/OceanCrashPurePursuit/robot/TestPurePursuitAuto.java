package OceanCrashPurePursuit.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import OceanCrashPurePursuit.autonomous.PurePursuitPath;
import OceanCrashPurePursuit.autonomous.waypoints.HeadingControlledWaypoint;
import OceanCrashPurePursuit.autonomous.waypoints.StopWaypoint;
import OceanCrashPurePursuit.autonomous.waypoints.Waypoint;
import OceanCrashPurePursuit.common.SimulatablePurePursuit;
import OceanCrashPurePursuit.common.math.Point;
import OceanCrashPurePursuit.common.math.Pose;
import OceanCrashPurePursuit.robot.util.MecanumPowers;
import OceanCrashPurePursuit.robot.util.MecanumUtil;
import OceanCrashPurePursuit.robot.CombinedRobot;
import org.openftc.revextensions2.RevBulkData;

import java.util.LinkedList;
import java.util.List;

@Autonomous(name = "TestPurePursuitAuto", group = "test")
public class TestPurePursuitAuto extends SimulatablePurePursuit{

    CombinedRobot robot;
    PurePursuitPath followPath;

    public final static double FIELD_RADIUS = 141 / 2.0; // in
    Pose DEFAULT_START_POSITION = new Pose(-FIELD_RADIUS + 22.75 + 9, FIELD_RADIUS - 9, 0);

    public List<Waypoint> getPurePursuitWaypoints() {
        LinkedList<Waypoint> scoreSkystones = Waypoint.collate(
                new Waypoint(DEFAULT_START_POSITION, 4)
        );
        scoreSkystones.addAll(Waypoint.collate(
                new Waypoint(DEFAULT_START_POSITION.x, 36, 3),
                new HeadingControlledWaypoint(DEFAULT_START_POSITION.x + 24, 36, 3, -Math.toRadians(90)),
                new HeadingControlledWaypoint(DEFAULT_START_POSITION.x, 36, 3, -Math.toRadians(90)),
                new HeadingControlledWaypoint(DEFAULT_START_POSITION.x, 10, 3, -Math.toRadians(90)),
                new StopWaypoint(DEFAULT_START_POSITION.x, 0, 8, Math.toRadians(90), 1)));

        return scoreSkystones;
    }

    @Override
    public void init() {
        Pose start = new Pose(-(141 / 2.0) + 22.75 + 9, (141 / 2.0) - 9, -Math.toRadians(90));
        this.robot = this.getRobot(start);

    }

    @Override
    public void start() {
        telemetry.clearAll();
        robot.initBulkReadTelemetry();
        followPath = new PurePursuitPath(robot, getPurePursuitWaypoints());
    }

    @Override
    public void loop() {
        robot.performBulkRead();
        robot.drawDashboardPath(followPath);
        robot.sendDashboardTelemetryPacket();

        if (!followPath.finished()) {
            followPath.update();
        } else {
            robot.setPowers(MecanumUtil.STOP);
            stop();
        }
    }

    @Override
    public void stop() {
        robot.setPowers(new MecanumPowers(0, 0, 0, 0));
    }
}
