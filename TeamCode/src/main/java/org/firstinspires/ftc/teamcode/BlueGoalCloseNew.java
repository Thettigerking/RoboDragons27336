package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue Close", group = "Autonomous")
@Configurable // Panels
public class BlueGoalCloseNew extends OpMode {
    private DcMotor Intake;
    private static CRServo BottomRampServo;
    private static CRServo BottomRampServo2;
    private static CRServo helperservo;
    private static CRServo helperservo2;
    private static Servo Pusher;
    private static Servo Pusher2;
    private Servo TiltControl;
    private DcMotor RightOuttake, LeftOuttake;


    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private BlueGoalCloseNew.Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public class Paths {


        private TelemetryManager panelsTelemetry; // Panels Telemetry instance
        public Follower follower; // Pedro Pathing follower instance
        private int pathState; // Current autonomous path state (state machine)
        private BlueGoalCloseAuto.Paths paths; // Paths defined in the Paths class
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
            init();
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(23.500, 127.000), new Pose(44.000, 107.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(324), Math.toRadians(311))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(44.000, 107.000), new Pose(44.000, 83.500))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(311), Math.toRadians(180))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(44.000, 83.500), new Pose(15.000, 83.500))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(15.000, 83.500), new Pose(44.000, 107.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(311))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(44.000, 107.000), new Pose(44.000, 60.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(311), Math.toRadians(180))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(44.000, 60.000), new Pose(15.000, 60.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(15.000, 60.000), new Pose(44.000, 107.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(311))
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(44.000, 107.000), new Pose(44.000, 35.600))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(311), Math.toRadians(180))
                    .build();

            Path9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(44.000, 35.600), new Pose(15.000, 35.600))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path10 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(15.000, 35.600), new Pose(44.000, 107.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(311))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        final double INTAKE_POWERING = -1.0;
        if (!follower.isBusy()) {

            switch (pathState) {

                case 0:
                    follower.followPath(paths.Path1);
                    pathState++;
                    break;

                case 1:
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
            }
        }

        return pathState;
    }
}