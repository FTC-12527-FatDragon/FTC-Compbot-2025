package com.example.meepmepp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import org.rowlandhall.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class MeepMeepTesting {
    public static Pose2d Basket = new Pose2d(9.5, 16.5, -45);
    // Ascent zone for Pick
    public static double xValue6 = 60;
    public static double yValue6 = -16;
    public static double heading6 = 270;
    public static double tangent6 = -300;
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

//        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep).setStartPose(new Pose2d(38, 71, 90))
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
//                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-56, -56, -30))
//                        .setReversed(false).splineToSplineHeading(new Pose2d(-23.44, -5, Math.toRadians(0)), Math.toRadians(0))
//                        .build());

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setStartPose(new Pose2d(38, 71, 90))
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70.5, 75, Math.toRadians(139.10), Math.toRadians(180), 13)
                .followTrajectorySequence(drive -> 
                    drive.trajectorySequenceBuilder(new Pose2d(-25.44, 0, 0)).setReversed(true)
                        .splineToLinearHeading(new Pose2d(-59, -54.5, Math.toRadians(60)), Math.toRadians(210))
                            .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}