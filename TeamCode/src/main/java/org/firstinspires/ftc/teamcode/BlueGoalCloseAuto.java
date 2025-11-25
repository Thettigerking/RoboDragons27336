package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name = "Blueclose")
public class BlueGoalCloseAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(-70, -10, 0);

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        waitForStart();

        if (opModeIsActive()) {
            Action actionName = drive.actionBuilder(startPose)
                    .strafeToLinearHeading(new Vector2d(-34, -34), Math.toRadians(0))
                    .turnTo(Math.toRadians(45))
                    .turnTo(Math.toRadians(0))
                    .strafeToLinearHeading(new Vector2d(-23, -47), Math.toRadians(0))
                    .strafeToLinearHeading(new Vector2d(-34, -34), Math.toRadians(0))
                    .turnTo(Math.toRadians(45))
                    .turnTo(Math.toRadians(0))
                    .strafeToLinearHeading(new Vector2d(0, -47), Math.toRadians(0))
                    .strafeToLinearHeading(new Vector2d(-34, -34), Math.toRadians(0))
                    .turnTo(Math.toRadians(45))
                    .turnTo(Math.toRadians(0))
                    .strafeToLinearHeading(new Vector2d(24, -47), Math.toRadians(0))
                    .strafeToLinearHeading(new Vector2d(-34, -34), Math.toRadians(0))
                    .turnTo(Math.toRadians(45))
                    .turnTo(Math.toRadians(0))
                    .strafeToLinearHeading(new Vector2d(38, 33), Math.toRadians(0))
                    .build();
            if (opModeIsActive()) {
                Actions.runBlocking(actionName);
            }
        }
    }
}


