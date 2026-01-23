package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;
// at the top of the file:


// you can then access the follower as follower()
@Autonomous(name = "BlueGoalCloseAuto", group = "Autonomous")
@Configurable // Panels
public class BlueGoalCloseAuto extends OpMode {
    private DcMotor Intake;
    private CRServo BottomRampServo, BottomRampServo2, helperservo, helperservo2;
    private Servo Pusher, Pusher2, TiltControl;
    private DcMotor RightOuttake, LeftOuttake;


    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    MotorGroup Outtake = new MotorGroup(
            new MotorEx("Left Motor Outtake"),
            new MotorEx("Right Motor Outtake")
    );


    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(33.5, 135.5, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine
        Intake = hardwareMap.get(DcMotor.class, "Intake");

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public class Paths {

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;
        public PathChain Path11;


        private void init() {

            BottomRampServo  = hardwareMap.get(CRServo.class, "BottomRampServo");
            BottomRampServo2 = hardwareMap.get(CRServo.class, "BottomRampServo2");
            helperservo = hardwareMap.get(CRServo.class, "helperservo");
            helperservo2 = hardwareMap.get(CRServo.class, "helperservo2");

            Pusher  = hardwareMap.get(Servo.class, "Pusher");
            Pusher2 = hardwareMap.get(Servo.class, "Pusher2");
            TiltControl = hardwareMap.get(Servo.class, "TiltControl");

            Intake = hardwareMap.get(DcMotor.class, "Intake");
            RightOuttake = hardwareMap.get(DcMotor.class, "Right Motor Outtake");
            LeftOuttake  = hardwareMap.get(DcMotor.class, "Left Motor Outtake");
        }


        public Paths(Follower follower) {

            init() ;

            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(34.000, 135.500), new Pose(59.000, 97.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(325))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(59.000, 97.000), new Pose(58.000, 97.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(325), Math.toRadians(180))
                    .build();

            new FollowPath(Path2, true, 0.5);
            //10
            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(42.000, 97.000), new Pose(21.000, 97.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(21.000, 97.000), new Pose(72.000, 72.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(315))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(72.000, 72.000), new Pose(42.000, 60.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(180))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(42.000, 60.000), new Pose(21.000, 60.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(21.000, 60.000), new Pose(72.000, 72.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(315))
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(72.000, 72.000), new Pose(42.000, 35.500))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(180))
                    .build();

            Path9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(42.000, 35.500), new Pose(21.000, 35.500))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path10 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(21.000, 35.500), new Pose(72.000, 72.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(315))
                    .build();

            Path11 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(72.000, 72.000), new Pose(105.000, 33.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(180))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        final double INTAKE_POWERING = -1.0;
        if (!follower.isBusy()) {

            switch (pathState) {

                case 0:
                    TiltControl.setPosition(0.325);
                    Pusher.setPosition(0.85);
                    follower.followPath(paths.Path1);
                    BottomRampServo.setPower(-1);
                    BottomRampServo2.setPower(-1);
                    helperservo.setPower(-1);
                    helperservo2.setPower(-1);
                    RightOuttake.setPower(-0.5);
                    LeftOuttake.setPower(-0.5);
                    pathState++;
                    break;

                case 1:
                    follower.followPath(paths.Path2);
                    Intake.setPower(INTAKE_POWERING);
                    RightOuttake.setPower(-0.465);
                    LeftOuttake.setPower(-0.465);

                    Pusher.setPosition(0.5);
                    try {
                        Thread.sleep(1750);
                    } catch(InterruptedException e) {
                        telemetry.addData("Warning","Sleeping interrupted:");
                    }
                    Pusher.setPosition(0.85);
                    try {
                        Thread.sleep(2250);
                    } catch(InterruptedException e) {
                        telemetry.addData("Warning","Sleeping interrupted:");
                    }
                    Pusher.setPosition(0.5);
                    try {
                        Thread.sleep(2250);
                    } catch(InterruptedException e) {
                        telemetry.addData("Warning","Sleeping interrupted:");
                    }
                    Pusher.setPosition(0.85);
                    try {
                        Thread.sleep(2250);
                    } catch(InterruptedException e) {
                        telemetry.addData("Warning","Sleeping interrupted:");
                    }
                    Pusher.setPosition(0.5);
                    try {
                        Thread.sleep(2250);
                    } catch(InterruptedException e) {
                        telemetry.addData("Warning","Sleeping interrupted:");
                    }
                    Pusher.setPosition(0.85);
                    try {
                        Thread.sleep(2250);
                    } catch(InterruptedException e) {
                        telemetry.addData("Warning","Sleeping interrupted:");
                    }
                    BottomRampServo.setPower(0);
                    BottomRampServo2.setPower(0);
                    helperservo.setPower(0);
                    helperservo2.setPower(0);
                    RightOuttake.setPower(0);
                    LeftOuttake.setPower(0);
                    Intake.setPower(0);
                    pathState++;
                    break;

                case 2:
                    follower.followPath(paths.Path3);
                    pathState++;
                    break;

                case 3:
                    follower.followPath(paths.Path4);
                    pathState++;
                    break;

                case 4:
                    follower.followPath(paths.Path5);
                    pathState++;
                    break;

                case 5:
                    follower.followPath(paths.Path6);
                    pathState++;
                    break;

                case 6:
                    follower.followPath(paths.Path7);
                    pathState++;
                    break;

                case 7:
                    follower.followPath(paths.Path8);
                    pathState++;
                    break;
                case 8:
                    follower.followPath(paths.Path9);
                    pathState++;
                    break;
                case 9:
                    follower.followPath(paths.Path10);
                    pathState++;
                    break;
                case 10:
                    follower.followPath(paths.Path11);
                    pathState++;
                    break;
            }
        }

        return pathState;
    }
    Command OuttakeCommand = new LambdaCommand()
            .setStart(() -> new SetPower(Outtake, -1))
            .setUpdate(() -> new SetPower(Outtake, -1))
            .setStop(interrupted -> new SetPower(Outtake, 0))
            .setIsDone(() -> true) // Returns if the command has finished
            .requires(/* subsystems the command implements */)
            .setInterruptible(true)
            .named("OuttakeCom");// sets the name of the command; optional
}