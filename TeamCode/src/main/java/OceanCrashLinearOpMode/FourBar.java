package OceanCrashLinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class FourBar {

    private CRServo spinL;
    private CRServo spinR;

    private Servo grab;

    private LinearOpMode opMode;

    public FourBar(LinearOpMode opMode) throws InterruptedException {

        this.opMode = opMode;

        spinL = this.opMode.hardwareMap.crservo.get("spinL");
        spinR = this.opMode.hardwareMap.crservo.get("spinR");

        grab = this.opMode.hardwareMap.servo.get("grab");

        spinL.setDirection(DcMotorSimple.Direction.REVERSE);
        spinR.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void grabCone() {
        grab.setPosition(1);
    }

    public void releaseCone() {
        grab.setPosition(0);
    }

}
