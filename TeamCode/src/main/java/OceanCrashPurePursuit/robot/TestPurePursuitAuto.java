package OceanCrashPurePursuit.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
import OceanCrashPurePursuit.common.math.Point;
import OceanCrashPurePursuit.common.math.Pose;
import OceanCrashPurePursuit.robot.util.MecanumPowers;
import OceanCrashPurePursuit.robot.util.MecanumUtil;
import OceanCrashPurePursuit.robot.CombinedRobot;
import org.openftc.revextensions2.RevBulkData;

import java.util.LinkedList;
import java.util.List;

@Autonomous(name = "TestPurePursuitAuto", group = "test")
public class TestPurePursuitAuto extends SimulatablePurePursuit {

    CombinedRobot robot;
    PurePursuitPath followPath;

    public final static double FIELD_RADIUS = 141 / 2.0; // in
    Pose DEFAULT_START_POSITION = new Pose(-FIELD_RADIUS + 12, FIELD_RADIUS - 24 - 12, 0);

    public List<Waypoint> getPurePursuitWaypoints() {
        LinkedList<Waypoint> scoreSkystones = Waypoint.collate(
                new Waypoint(DEFAULT_START_POSITION, 4)
        );
        scoreSkystones.addAll(Waypoint.collate(
                //preload
                new HeadingControlledWaypoint(DEFAULT_START_POSITION.x + 5, DEFAULT_START_POSITION.y, 3, -Math.toRadians(45), new Start()),
                new HeadingControlledWaypoint(DEFAULT_START_POSITION.x + 22, DEFAULT_START_POSITION.y, 3, -Math.toRadians(45), new ReadyToDeposit()),
                new HeadingControlledWaypoint(DEFAULT_START_POSITION.x + 22.5, DEFAULT_START_POSITION.y - 22.5, 3, -Math.toRadians(45)),
                new StopWaypoint(DEFAULT_START_POSITION.x + 22.5, DEFAULT_START_POSITION.y - 22.5, 3, -Math.toRadians(45), .5, new Deposit()),

                //grab 1
                new HeadingControlledWaypoint(DEFAULT_START_POSITION.x + 52, DEFAULT_START_POSITION.y - 24, 3, Math.toRadians(90)),
                new HeadingControlledWaypoint(DEFAULT_START_POSITION.x + 52, DEFAULT_START_POSITION.y - 10, 3, Math.toRadians(90), new ReadyToGrab()),
                new HeadingControlledWaypoint(DEFAULT_START_POSITION.x + 51, DEFAULT_START_POSITION.y + 8, 3, Math.toRadians(90), new ReadyToGrab()),
                new StopWaypoint(DEFAULT_START_POSITION.x + 51, DEFAULT_START_POSITION.y + 8, 3, Math.toRadians(90), 1, new Grab(150)),
                //deposit 1
                new HeadingControlledWaypoint(DEFAULT_START_POSITION.x + 52, DEFAULT_START_POSITION.y - 10, 3, Math.toRadians(90)),
                new HeadingControlledWaypoint(DEFAULT_START_POSITION.x + 52, DEFAULT_START_POSITION.y - 23, 3, -Math.toRadians(135), new ReadyToDeposit()),
                new StopWaypoint(DEFAULT_START_POSITION.x + 51, DEFAULT_START_POSITION.y - 20, 3, -Math.toRadians(135), .5, new TrollDeposit()),
                //new StopWaypoint(DEFAULT_START_POSITION.x + 51, DEFAULT_START_POSITION.y - 20, 3, -Math.toRadians(135), .5, new WaitSubroutine(1000)),


                new HeadingControlledWaypoint(DEFAULT_START_POSITION.x + 52, DEFAULT_START_POSITION.y - 24, 3, Math.toRadians(90)),
                new HeadingControlledWaypoint(DEFAULT_START_POSITION.x + 52, DEFAULT_START_POSITION.y - 10, 3, Math.toRadians(90), new ReadyToGrab()),
                new HeadingControlledWaypoint(DEFAULT_START_POSITION.x + 52, DEFAULT_START_POSITION.y + 8, 3, Math.toRadians(90), new ReadyToGrab()),
                new StopWaypoint(DEFAULT_START_POSITION.x + 52, DEFAULT_START_POSITION.y + 8, 3, Math.toRadians(90), 1, new Grab(150))

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
