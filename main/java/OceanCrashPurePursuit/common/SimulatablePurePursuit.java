package OceanCrashPurePursuit.common;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import OceanCrashPurePursuit.common.math.Pose;
import OceanCrashPurePursuit.robot.CombinedRobot;

public abstract class SimulatablePurePursuit extends OpMode {
    Pose DEFAULT_START_POSE = new Pose(0, 0, 0);

    // For autonomous

    public CombinedRobot getRobot(Pose start) {
        return new CombinedRobot(this.hardwareMap, this.telemetry, FtcDashboard.getInstance(), start);
    }

    public void stop() {
        requestOpModeStop();
    }

    public CombinedRobot getRobot() {
        return this.getRobot(DEFAULT_START_POSE);
    }
}
