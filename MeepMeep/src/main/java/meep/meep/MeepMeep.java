package meep.meep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeep {
    public static void main(String[] args) {
        com.noahbres.meepmeep.MeepMeep meepMeep = new com.noahbres.meepmeep.MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(35, 35, Math.toRadians(180), Math.toRadians(180), 13)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -61, Math.toRadians(90)))
                                //preload
                                .splineToSplineHeading(new Pose2d(-34.3, -33.3, Math.toRadians(90)), Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(-32.3, -15.8, Math.toRadians(65)), Math.toRadians(65))

                                //grab 1
                                .splineToSplineHeading(new Pose2d(-34.3, -12, Math.toRadians(180)), Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(-47.6, -12, Math.toRadians(180)), Math.toRadians(180))
                                //deposit 1
                                .splineToSplineHeading(new Pose2d(-31.1, -15.9, Math.toRadians(65)), Math.toRadians(65))

                                //grab 2
                                .splineToSplineHeading(new Pose2d(-34.3, -12, Math.toRadians(180)), Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(-47.6, -12, Math.toRadians(180)), Math.toRadians(180))
                                //deposit 2
                                .splineToSplineHeading(new Pose2d(-31.1, -15.9, Math.toRadians(65)), Math.toRadians(65))

                                //grab 3
                                .splineToSplineHeading(new Pose2d(-34.3, -12, Math.toRadians(180)), Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(-47.6, -12, Math.toRadians(180)), Math.toRadians(180))
                                //deposit 3
                                .splineToSplineHeading(new Pose2d(-31.1, -15.9, Math.toRadians(65)), Math.toRadians(65))

                                //grab 4
                                .splineToSplineHeading(new Pose2d(-34.3, -12, Math.toRadians(180)), Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(-47.6, -12, Math.toRadians(180)), Math.toRadians(180))
                                //deposit 4
                                .splineToSplineHeading(new Pose2d(-31.1, -15.9, Math.toRadians(65)), Math.toRadians(65))

                                //park
                                .lineToLinearHeading(new Pose2d(-18, -15.9, Math.toRadians(-90)))


                                .build()
                );

        meepMeep.setBackground(com.noahbres.meepmeep.MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}