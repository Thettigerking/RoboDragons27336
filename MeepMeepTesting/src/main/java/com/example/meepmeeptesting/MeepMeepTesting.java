package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

/*     RoadRunnerBotEntity redBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(Constants.ROBOT_WIDTH, Constants.ROBOT_HEIGHT)
                .setConstraints(Constants.MAX_VEL, Constants.MAX_ACCEL, Constants.MAX_ANG_VEL, Constants.MAX_ANG_ACCEL, Constants.TRACK_WIDTH)
                .build();
*/

        RoadRunnerBotEntity blueBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(Constants.ROBOT_WIDTH, Constants.ROBOT_HEIGHT)
                .setConstraints(Constants.MAX_VEL, Constants.MAX_ACCEL, Constants.MAX_ANG_VEL, Constants.MAX_ANG_ACCEL, Constants.TRACK_WIDTH)
                .build();


        blueBot.runAction(blueBot.getDrive().actionBuilder(new Pose2d(-70, -10, Math.toRadians(0)))


                //.strafeToLinearHeading(new Vector2d(35.8, 22.4), Math.toRadians(90))
                //.strafeToConstantHeading(new Vector2d(35.8, 52.5))
                .strafeToLinearHeading(new Vector2d(-34, -34), Math.toRadians(0))
                                .turnTo(Math.toRadians(-135))
                .turnTo(Math.toRadians(0))
//      .strafeToLinearHeading(new Vector2d(-34, -34), Math.toRadians(0))
                //.strafeToLinearHeading(new Vector2d(-23, -47), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-23, -47), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-34, -34), Math.toRadians(0))
                .turnTo(Math.toRadians(-135))
                .turnTo(Math.toRadians(0))
                //.strafeToLinearHeading(new Vector2d(-34, -34), Math.toRadians(0))
                //.strafeToLinearHeading(new Vector2d(-34, -34), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(0, -47), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-34, -34), Math.toRadians(0))
                .turnTo(Math.toRadians(-135))
                .turnTo(Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(24, -47), Math.toRadians(0))
//                .turnTo(Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-34, -34), Math.toRadians(0))
                .turnTo(Math.toRadians(-135))
                .turnTo(Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(38, -33), Math.toRadians(0))
//                .turnTo(Math.toRadians(-135))
//
//                .strafeToLinearHeading(new Vector2d(56, -9), Math.toRadians(0))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
//                .addEntity(redBot)
                .addEntity(blueBot)
                //.addEntity(botMotif1)
                //.addEntity(botMotif2)
                //.addEntity(botMotif3)
                .start();
    }
}