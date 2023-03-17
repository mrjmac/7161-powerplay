package meep.meep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeep {
    public static void main(String[] args) {
        com.noahbres.meepmeep.MeepMeep meepMeep = new com.noahbres.meepmeep.MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(35, 35, Math.toRadians(180), Math.toRadians(180), 13)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-72, 35.3 - (3.0/8), 0))
                                //preload
                                .splineToSplineHeading(new Pose2d(-23.9, 33.5, Math.toRadians(-25)), Math.toRadians(-15))


                                //grab 1
                                .splineToSplineHeading(new Pose2d(-19.7, 34.3, Math.toRadians(90)), Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(-19.7, 48.0), Math.toRadians(90))
                                //deposit 1
                                .splineToConstantHeading(new Vector2d(-22.2, 15.3), Math.toRadians(85))
                                .splineToSplineHeading(new Pose2d(-21.2, 9, Math.toRadians(45)), Math.toRadians(100))

                                //grab 2
                                //.splineToSplineHeading(new Pose2d(-17.7, 34.3, Math.toRadians(90)), Math.toRadians(90))
                                //.splineToConstantHeading(new Vector2d(-17.7, 48.0), Math.toRadians(90))
                                .build()
                );

        meepMeep.setBackground(com.noahbres.meepmeep.MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}