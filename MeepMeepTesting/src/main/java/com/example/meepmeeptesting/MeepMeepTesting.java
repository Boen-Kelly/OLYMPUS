package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 30, Math.toRadians(180), Math.toRadians(180), 13.85)
                .setStartPose(new Pose2d(-31.425,64.75, Math.toRadians(-90)))
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-31.425,64.75, Math.toRadians(-90)))
                                .lineToConstantHeading(new Vector2d(-32, 64.75))
                                .splineToConstantHeading(new Vector2d(-34, 62.75), Math.toRadians(-90))
                                .lineToLinearHeading(new Pose2d(-34,3, Math.toRadians(-90)))
                                .lineToLinearHeading(new Pose2d(-34, 7.5, Math.toRadians(-90)))
                                .lineToSplineHeading(new Pose2d(-34,12, Math.toRadians(-220)))
                                .back(10)
                                .forward(10)
                                .splineTo(new Vector2d(-40, 12), Math.toRadians(185))
                                .forward(23)
                                .lineToSplineHeading(new Pose2d(-40,12, Math.toRadians(180)))
                                .splineTo(new Vector2d(-36,12), Math.toRadians(-40))
                                .back(10)
                                .lineToLinearHeading(new Pose2d(-34,12, Math.toRadians(0)))
//                                .splineToConstantHeading(new Vector2d(-40,12), Math.toRadians(0))
//                                .splineToSplineHeading(new Pose2d(-40,12, Math.toRadians(0)), Math.toRadians(0))
                                .lineToLinearHeading(new Pose2d(-60,12, Math.toRadians(0)))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}