package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTestingFarSide {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity redBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(Constants.ROBOT_WIDTH, Constants.ROBOT_HEIGHT)
                .setConstraints(Constants.MAX_VEL, Constants.MAX_ACCEL, Constants.MAX_ANG_VEL, Constants.MAX_ANG_ACCEL, Constants.TRACK_WIDTH)
                .build();

        RoadRunnerBotEntity blueBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(Constants.ROBOT_WIDTH, Constants.ROBOT_HEIGHT)
                .setConstraints(Constants.MAX_VEL, Constants.MAX_ACCEL, Constants.MAX_ANG_VEL, Constants.MAX_ANG_ACCEL, Constants.TRACK_WIDTH)
                .build();

        redBot.runAction(redBot.getDrive().actionBuilder(new Pose2d(62, -20, Math.toRadians(0)))

                .strafeToLinearHeading(new Vector2d(-34, -20), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-34, -34), Math.toRadians(0))
                .turnTo(Math.toRadians(45))
                .turnTo(Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-34, -20), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(62, -20), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(62, -54), Math.toRadians(0))
                .turnTo(Math.toRadians(-90))
                .turnTo(Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(62, -20), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-34, -20), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-34, -34), Math.toRadians(0))
                .turnTo(Math.toRadians(45))
                .turnTo(Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(38, 33), Math.toRadians(0))
                .build());

        blueBot.runAction(blueBot.getDrive().actionBuilder(new Pose2d(62, 20, Math.toRadians(0)))

                .strafeToLinearHeading(new Vector2d(-34, 20), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-34, 34), Math.toRadians(0))
                .turnTo(Math.toRadians(-45))
                .turnTo(Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-34, 20), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(62, 20), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(62, 54), Math.toRadians(0))
                .turnTo(Math.toRadians(90))
                .turnTo(Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(62, 20), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-34, 20), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-34, 34), Math.toRadians(0))
                .turnTo(Math.toRadians(-45))
                .turnTo(Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(38, -33), Math.toRadians(0))
                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(redBot)
                .addEntity(blueBot)
                .start();
    }
}