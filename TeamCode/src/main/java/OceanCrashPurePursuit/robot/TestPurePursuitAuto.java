package OceanCrashPurePursuit.robot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import OceanCrashPurePursuit.autonomous.PurePursuitPath;
import OceanCrashPurePursuit.autonomous.waypoints.Deposit;
import OceanCrashPurePursuit.autonomous.waypoints.Grab;
import OceanCrashPurePursuit.autonomous.waypoints.HeadingControlledWaypoint;
import OceanCrashPurePursuit.autonomous.waypoints.PointTurnWaypoint;
import OceanCrashPurePursuit.autonomous.waypoints.ReadyToDeposit;
import OceanCrashPurePursuit.autonomous.waypoints.ReadyToGrab;
import OceanCrashPurePursuit.autonomous.waypoints.Start;
import OceanCrashPurePursuit.autonomous.waypoints.StopWaypoint;
import OceanCrashPurePursuit.autonomous.waypoints.TrollDeposit;
import OceanCrashPurePursuit.autonomous.waypoints.WaitSubroutine;
import OceanCrashPurePursuit.autonomous.waypoints.Waypoint;
import OceanCrashPurePursuit.common.SimulatablePurePursuit;
import OceanCrashPurePursuit.common.math.Pose;
import OceanCrashPurePursuit.robot.util.MecanumPowers;
import OceanCrashPurePursuit.robot.util.MecanumUtil;

import java.util.LinkedList;
import java.util.List;

@Autonomous(name = "TestPurePursuitAuto", group = "test")
public class TestPurePursuitAuto extends SimulatablePurePursuit {

    CombinedRobot robot;
    PurePursuitPath followPath;

    public final static double FIELD_RADIUS = 141 / 2.0; // in
    Pose DEFAULT_START_POSITION = new Pose(-FIELD_RADIUS + 8, FIELD_RADIUS - 24 - 12, 0);

    public List<Waypoint> getPurePursuitWaypoints() {
        LinkedList<Waypoint> scoreSkystones = Waypoint.collate(
                new Waypoint(DEFAULT_START_POSITION, 4)
        );
        scoreSkystones.addAll(Waypoint.collate(
                //preload
                new HeadingControlledWaypoint(DEFAULT_START_POSITION.x + 5, DEFAULT_START_POSITION.y, 2, Math.toRadians(0), new Start()),
                new HeadingControlledWaypoint(DEFAULT_START_POSITION.x + 28, DEFAULT_START_POSITION.y, 2, Math.toRadians(0), new ReadyToDeposit()),
                new HeadingControlledWaypoint(DEFAULT_START_POSITION.x + 47, DEFAULT_START_POSITION.y + 2, 2, -Math.toRadians(45)),
                new StopWaypoint(DEFAULT_START_POSITION.x + 47.5, DEFAULT_START_POSITION.y + 3, 2, -Math.toRadians(45), 1, new Deposit()),

                //grab 1
                new PointTurnWaypoint(DEFAULT_START_POSITION.x + 50.5, DEFAULT_START_POSITION.y - 4, 1, Math.toRadians(90), Math.toRadians(5),new ReadyToGrab()),
                new StopWaypoint(DEFAULT_START_POSITION.x + 51, DEFAULT_START_POSITION.y + 10, 1, Math.toRadians(90), 1, new Grab(100)),
                //deposit 1
                new HeadingControlledWaypoint(DEFAULT_START_POSITION.x + 47, DEFAULT_START_POSITION.y + 1, 2, -Math.toRadians(45), new ReadyToDeposit()),
                new StopWaypoint(DEFAULT_START_POSITION.x + 47.5, DEFAULT_START_POSITION.y + .5, 2, -Math.toRadians(45), 1, new TrollDeposit())


        ));


        return scoreSkystones;
    }


    @Override
    public void init() {
        Pose start = DEFAULT_START_POSITION;
        this.robot = this.getRobot(start);
        robot.grab();

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
