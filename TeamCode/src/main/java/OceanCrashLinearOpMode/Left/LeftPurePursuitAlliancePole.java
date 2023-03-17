package OceanCrashLinearOpMode.Left;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

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
import OceanCrashPurePursuit.robot.CombinedRobot;
import OceanCrashPurePursuit.robot.util.MecanumPowers;
import OceanCrashPurePursuit.robot.util.MecanumUtil;

import java.util.LinkedList;
import java.util.List;
@Disabled
@Autonomous(name = "pp", group = "Left")
public class LeftPurePursuitAlliancePole extends SimulatablePurePursuit {

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

                new HeadingControlledWaypoint(DEFAULT_START_POSITION.x + 15, DEFAULT_START_POSITION.y, 2, Math.toRadians(0), new Start()),
                new HeadingControlledWaypoint(DEFAULT_START_POSITION.x + 28, DEFAULT_START_POSITION.y + 4, 2, Math.toRadians(0), new ReadyToDeposit()),
                new HeadingControlledWaypoint(DEFAULT_START_POSITION.x + 44.1, DEFAULT_START_POSITION.y + 4, 2, -Math.toRadians(45)),
                new StopWaypoint(DEFAULT_START_POSITION.x + 44.1, DEFAULT_START_POSITION.y + 4, 2, -Math.toRadians(45), 1, new Deposit()),

                //grab 1
                //new HeadingControlledWaypoint(DEFAULT_START_POSITION.x + 52, DEFAULT_START_POSITION.y - 24, 2, Math.toRadians(90)),
                //new HeadingControlledWaypoint(DEFAULT_START_POSITION.x + 52, DEFAULT_START_POSITION.y - 10, 2, Math.toRadians(90), new ReadyToGrab()),
                new HeadingControlledWaypoint(DEFAULT_START_POSITION.x + 51, DEFAULT_START_POSITION.y + 7, 2, Math.toRadians(90), new ReadyToGrab()),
                new StopWaypoint(DEFAULT_START_POSITION.x + 51, DEFAULT_START_POSITION.y + 7, 2, Math.toRadians(90), 1, new Grab(150)),
                //deposit 1
                /*
                new HeadingControlledWaypoint(DEFAULT_START_POSITION.x + 52, DEFAULT_START_POSITION.y - 4, 2, Math.toRadians(90)),
                new HeadingControlledWaypoint(DEFAULT_START_POSITION.x + 51, DEFAULT_START_POSITION.y - 15, 2, Math.toRadians(225), new ReadyToDeposit()),
                new HeadingControlledWaypoint(DEFAULT_START_POSITION.x + 53, DEFAULT_START_POSITION.y - 24, 2, Math.toRadians(225)),
                new PointTurnWaypoint(DEFAULT_START_POSITION.x + 54, DEFAULT_START_POSITION.y - 25, 1.5, Math.toRadians(225), Math.toRadians(5), new TrollDeposit()),
                /*
                //grab 2
                new HeadingControlledWaypoint(DEFAULT_START_POSITION.x + 50, DEFAULT_START_POSITION.y - 24, 2, Math.toRadians(90)),
                new HeadingControlledWaypoint(DEFAULT_START_POSITION.x + 50, DEFAULT_START_POSITION.y - 10, 2, Math.toRadians(90), new ReadyToGrab()),
                new HeadingControlledWaypoint(DEFAULT_START_POSITION.x + 47.5, DEFAULT_START_POSITION.y + 6, 2, Math.toRadians(90), new ReadyToGrab()),
                new StopWaypoint(DEFAULT_START_POSITION.x + 48, DEFAULT_START_POSITION.y + 6, 1.5, Math.toRadians(90), 1, new Grab(120)),
                //deposit 2
                new PointTurnWaypoint(DEFAULT_START_POSITION.x + 47, DEFAULT_START_POSITION.y + 12, 2, Math.toRadians(260), Math.toRadians(20)),
                new HeadingControlledWaypoint(DEFAULT_START_POSITION.x + 47, DEFAULT_START_POSITION.y + 5, 2, -Math.toRadians(45), new ReadyToDeposit()),
                new PointTurnWaypoint(DEFAULT_START_POSITION.x + 47, DEFAULT_START_POSITION.y + 4, 1.5, -Math.toRadians(45), Math.toRadians(10), new TrollDeposit()),

                /*
                //grab 3
                new HeadingControlledWaypoint(DEFAULT_START_POSITION.x + 46, DEFAULT_START_POSITION.y - 24, 2, Math.toRadians(90)),
                new HeadingControlledWaypoint(DEFAULT_START_POSITION.x + 46, DEFAULT_START_POSITION.y - 10, 2, Math.toRadians(90), new ReadyToGrab()),
                new HeadingControlledWaypoint(DEFAULT_START_POSITION.x + 46, DEFAULT_START_POSITION.y + 10, 2, Math.toRadians(90), new ReadyToGrab()),
                */
                new StopWaypoint(DEFAULT_START_POSITION.x + 46, DEFAULT_START_POSITION.y + 10, 2, Math.toRadians(90), 1, new Grab(100))

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
