package OceanCrashLinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class testLift {

    private final DcMotor liftL; // [E2]
    private final DcMotor liftR; // [C2]

    private final LinearOpMode opMode;

    private TouchSensor touch;

    private final double STALL_POWER = -0.0005;

    public testLift(LinearOpMode opMode) throws InterruptedException {

        this.opMode = opMode;

        liftL = this.opMode.hardwareMap.dcMotor.get("liftL"); // [E2]
        liftR = this.opMode.hardwareMap.dcMotor.get("liftR"); // [C2]

        liftL.setDirection(DcMotorSimple.Direction.FORWARD);
        liftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftL.setTargetPosition(0);

        liftR.setDirection(DcMotorSimple.Direction.REVERSE);
        liftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftR.setTargetPosition(0);


        touch = this.opMode.hardwareMap.touchSensor.get("touch");
    }

    public void setPosition(int position)
    {
        liftL.setTargetPosition(position);
        liftR.setTargetPosition(position);
    }

}
