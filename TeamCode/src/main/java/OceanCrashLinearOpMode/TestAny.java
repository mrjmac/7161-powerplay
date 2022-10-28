package OceanCrashLinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Test", group = "Test")
public class TestAny extends LinearOpMode {

    private Drivetrain drivetrain;
    private Vision vision;
    private Lift lift;
    private Intake intake;

    private final double turnP45 = .43;
    private final double turnD45 = .40;

    private final double turnP135 = .24;
    private final double turnD135 = .20;

    private final double moveP48 = .442;
    private final double moveP4 = 1;
    private final double moveP20 = .432;

    private int pos;

    @Override
    public void runOpMode() throws InterruptedException {

        drivetrain = new Drivetrain(this);
        vision = new Vision(this);
        lift = new Lift(this);
        intake = new Intake(this);

        while(!isStarted()){
            pos = vision.getPark();
            telemetry.addData("park: ", pos);
            telemetry.update();
        }

        telemetry.addData("park: ", pos);
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;
        while (!isStopRequested()) {

//=============================================== DT ===============================================//

            drivetrain.gyroInch(moveP48, 50.5, 2, 0);
            sleep(500);
            drivetrain.turnPD(45, turnP45, turnD45, 2);
            //sleep(500);
            //drivetrain.gyroInch(moveP4, 4, 2, 45);
            //sleep(500);
            //drivetrain.gyroInch(-moveP4, 4, 2, 45);
            sleep(500);
            drivetrain.gyroInch(-moveP4, 2.5, 1, 45);
            sleep(500);
            drivetrain.turnPD(-90, turnP135, turnD135, 2);
            sleep(500);
            drivetrain.gyroInch(moveP20, 20, 2, -90);
            sleep(500);
            for (int i = 0; i < 2; i++)
            {
                drivetrain.gyroInch(-moveP48, 46, 2, -90);
                sleep(500);
                drivetrain.turnPD(-45, turnP45, turnD45, 2);
                sleep(500);
                drivetrain.turnPD(-90, turnP45, turnD45, 2);
                sleep(500);
                drivetrain.gyroInch(moveP48, 41.5, 2, -90);
                sleep(500);
            }


            /*drivetrain.turnPD(45, .45, .03, 2); //45 = .45, .03.  135 = .27, .02
            sleep(2000);
            drivetrain.turnPD(0, .45, .03, 2);
            sleep(2000);
            */
            /*
            lift.grab();
            lift.setLiftPos(1300);
            sleep(10000);*/
           /*
            lift.resetLift(.5);
            sleep(10000);
            lift.setLiftPower(2000);
            lift.extendFourBar();
            sleep(500);
            lift.release();
            sleep(500);
            lift.grab();
            sleep(500);
            lift.retractFourBar();
            sleep(500);
            lift.resetLift(.5);/*
            //drivetrain.gyroInch(.442, 40, 5, 0); //.442 for 48 in, 1 for 4 in, .432 for 20 in, .x for 40 in
            //drivetrain.startMotors(.7, .7, .7, .7);
            //sleep(10000);
            //24, 10

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

            /*
            //set intake for low junction
            lift.setLift(1, 0.5);
            sleep(500);
            lift.resetLift(1);
            sleep(500);

             */

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
          /*  if (lift.getLiftPos() > 1300)
                lift.setLiftPower(-.0005);
                sleep(1000);

           */
            break;
        }
    }
}
