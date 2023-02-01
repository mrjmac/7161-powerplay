package OceanCrashPurePursuit.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import OceanCrashPurePursuit.autonomous.PurePursuitPath;
import OceanCrashPurePursuit.autonomous.waypoints.StopWaypoint;
import OceanCrashPurePursuit.autonomous.waypoints.Waypoint;
import OceanCrashPurePursuit.common.SimulatablePurePursuit;
import OceanCrashPurePursuit.common.math.Pose;
import OceanCrashPurePursuit.robot.util.MecanumPowers;
import OceanCrashPurePursuit.robot.util.MecanumUtil;
import OceanCrashPurePursuit.robot.CombinedRobot;
import org.openftc.revextensions2.RevBulkData;

import java.util.LinkedList;
import java.util.List;

public class TestPurePursuitAuto extends SimulatablePurePursuit{

    CombinedRobot robot;
    PurePursuitPath followPath;

    public List<Waypoint> getPurePursuitWaypoints() {
        return  Waypoint.collate(
                new Waypoint(new Pose(0, 0, 0), 4),
                new StopWaypoint(0, -38.5, 20, 1.5 * Math.PI, 0)
        );
    }

    @Override
    public void init() {
        Pose start = new Pose(0, 0, 0);
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
        RevBulkData data = robot.performBulkRead();
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
