package OceanCrashLinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Test", group = "Test")
public class TestAny extends LinearOpMode {

    private Drivetrain drivetrain;
    //private Vision vision;
    private Lift lift;
    private Intake intake;

    private int pos;

    @Override
    public void runOpMode() throws InterruptedException {

        drivetrain = new Drivetrain(this);
        //vision = new Vision(this);
        lift = new Lift(this);
        intake = new Intake(this);

        while(!isStarted()){
            //pos = vision.getPark();
            //telemetry.addData("park: ", pos);
            //telemetry.update();
        }

        //telemetry.addData("park: ", pos);
        //telemetry.update();

        waitForStart();

        if (isStopRequested()) return;
        while (!isStopRequested()) {

//=============================================== DT ===============================================//
            /*
            //go forwards
            drivetrain.gyroInch(1, 5, 5, 0);
            sleep(1000);

            //strafe left
            drivetrain.leftGyroStrafe(1, 5, 5, 0);
            sleep(1000);

            //strafe right
            drivetrain.rightGyroStrafe(1, 5, 5, 0);
            sleep(1000);

            //turn clockwise
            drivetrain.turnPD(90, .5, 0, 5);
            sleep(1000);

            //turn back
            drivetrain.turnPD(0, .5, 0, 5);
            sleep(1000);

             */

//=============================================== INTAKE ===============================================//
            /*
            //start intake
            intake.startIntake(1);
            sleep(1000);

            //reverse intake
            intake.startIntake(-1);
            sleep(1000);

            //stop intake
            intake.startIntake(0);
            sleep(1000);

             */

//=============================================== LIFT ===============================================//
            //extend four bar
            //lift.extendFourBar();

            //set intake for low junction
            lift.setLift(1, 0.5);
            sleep(500);
            lift.resetLift(1);
            sleep(500);

            /*
            //set intake for mid junction
            lift.setLift(2, 1);
            sleep(500);
            lift.resetLift(1);
            sleep(500);

            //set intake for high junction
            lift.setLift(3, 1);
            sleep(500);
            lift.resetLift(1);
            sleep(500);

            //set intake for 1st auto cycle
            lift.setLiftForCone(1, 1);
            sleep(300);
            lift.resetLift(1);
            sleep(500);

            //set intake for 2nd auto cycle
            lift.setLiftForCone(2, 1);
            sleep(300);
            lift.resetLift(1);

             */

        }
    }
}
