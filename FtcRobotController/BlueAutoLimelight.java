package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue Auto Testing", group = "Autonomous")
@Configurable // Panels
public class BlueAutoLimelight extends OpMode {
    private double outtakespeed = 890;
    private DcMotor Intake;
    private CRServo BottomRampServo, BottomRampServo2, helper3;
    private Servo Pusher, TiltControl;
    private DcMotorEx RightOuttake, LeftOuttake;
    private Limelight3A limelight;
    private DcMotor leftFront, leftBack, rightFront, rightBack;

    //_____________________________________________________
    private double distance;
    private double speed;
    private double speedx;

    private boolean align = false;
    private ElapsedTime aimTimer = new ElapsedTime();

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    //131
    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
//
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(8.5, 8.5, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        BottomRampServo  = hardwareMap.get(CRServo.class, "BottomRampServo");
        BottomRampServo2 = hardwareMap.get(CRServo.class, "BottomRampServo2");
        helper3 = hardwareMap.get(CRServo.class, "helper3");

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack  = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront= hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        Pusher  = hardwareMap.get(Servo.class, "Pusher");
        TiltControl = hardwareMap.get(Servo.class, "TiltControl");

        RightOuttake = hardwareMap.get(DcMotorEx.class, "Right Motor Outtake");
        LeftOuttake  = hardwareMap.get(DcMotorEx.class, "Left Motor Outtake");

        LeftOuttake.setDirection(DcMotorSimple.Direction.REVERSE);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);
        RightOuttake.setVelocityPIDFCoefficients(
                8.0,   // P
                0.0,    // I
                0.6,    // D
                13.5    // F (THIS MATTERS)
        );

        LeftOuttake.setVelocityPIDFCoefficients(
                8.0,
                0.0,
                0.6,
                13.5
        );
        RightOuttake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftOuttake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftOuttake.setDirection(DcMotorSimple.Direction.REVERSE);

        limelight.pipelineSwitch(0);
        limelight.start();
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();
        Intake.setPower(-1);
        LeftOuttake.setDirection(DcMotor.Direction.FORWARD);
        RightOuttake.setDirection(DcMotor.Direction.REVERSE);
        follower.update(); // Update Pedro Pathing
        try {
            pathState = autonomousPathUpdate(); // Update autonomous state machine
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        telemetry.addData("Ta:",result.getTa());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain Path1;
        public double Wait13;
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
        public PathChain Path12;
        public PathChain Path13;

        public PathChain Path15;
        public PathChain Path14;

        public Paths(Follower follower) {
            Path15 =  follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(8.5, 8.5), new Pose(25, 131))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(324))
                    .build();

            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(25, 131), new Pose(54.000, 90.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(324), Math.toRadians(318))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(54.000, 90.000), new Pose(48.000, 84.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(318), Math.toRadians(180))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(48.000, 84.000), new Pose(15, 84.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(15, 84.000), new Pose(35.000, 79.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(35.000, 79.000), new Pose(16.750, 75.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(16.750, 75.000), new Pose(54.000, 90.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(312))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(54.000, 90.000), new Pose(48.000, 61))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(312), Math.toRadians(180))
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(48.000, 61), new Pose(8.5000, 59))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
            Path9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(8.5, 59), new Pose(20, 61))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path13 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(20, 61), new Pose(54.000, 90.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(326))
                    .build();

            Path10 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(54.000, 90.000), new Pose(41.000, 36.500))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(326), Math.toRadians(180))
                    .build();

            Path11 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(41.000, 36.500), new Pose(8, 36.500))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path12 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(8, 36.500), new Pose(54.000, 90.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(326))
                    .build();
            Path14 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(54, 90), new Pose(24, 88.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(326), Math.toRadians(90))
                    .build();

        }
    }

    public int autonomousPathUpdate() throws InterruptedException {

        if (!follower.isBusy()) {
            switch (pathState) {


                case 0:

                    TiltControl.setPosition(0.4);
                    Pusher.setPosition(0.47);
                    RightOuttake.setVelocity(outtakespeed);
                    LeftOuttake.setVelocity(outtakespeed);
                    //RightOuttake.setVelocity(-860);
                    //LeftOuttake.setVelocity(-860);
                    Intake.setPower(-1);
                    follower.followPath(paths.Path1);

                    pathState++;
                    break;
                case 1:
                    align = false;
                    Pusher.setPosition(0.1);
                    try {
                        Thread.sleep(50);
                    } catch(InterruptedException e) {
                        telemetry.addData("Warning","Sleeping interrupted:");
                    }
                    aimTimer.reset();
                    BottomRampServo.setPower(-1);
                    BottomRampServo2.setPower(-1);
                    helper3.setPower(1);
                    align = false;
                    follower.pausePathFollowing();
                    while(!align) {
                        LLResult result = limelight.getLatestResult();
                        double tx = result.getTx();   // Limelight angle error

                        // ---- TUNING VALUES ----
                        double kP = 0.02;             // proportional gain
                        double minPower = 0.08;       // minimum turn power
                        double maxPower = 0.30;       // max turn power
                        double deadband = 0.1;        // degrees allowed error

                        if (Math.abs(tx) > deadband) {

                            double turnPower = tx * kP;

                            // Clamp to max power
                            turnPower = Math.max(-maxPower, Math.min(maxPower, turnPower));

                            // Enforce minimum power
                            if (Math.abs(turnPower) < minPower) {
                                turnPower = Math.signum(turnPower) * minPower;
                            }

                            // Apply turn
                            leftFront.setPower(turnPower);
                            leftBack.setPower(turnPower);
                            rightFront.setPower(-turnPower);
                            rightBack.setPower(-turnPower);

                        } else {
                            // Aligned
                            leftFront.setPower(0);
                            leftBack.setPower(0);
                            rightFront.setPower(0);
                            rightBack.setPower(0);
                            align = true;
                        }
                    }
                    follower.resumePathFollowing();
                    while (aimTimer.milliseconds() < 2100) {
                        LLResult result = limelight.getLatestResult();
                        distance = distancem(result.getTa());
                        speedx = (-0.0000182763 * (distance * distance)) + (0.003602 * distance) - 0.0113504;
                        //speed = (4.4 * distance) + 740;
                        speed = (0.0061376 * (distance * distance)) + (2.66667 * distance) + 850.7619;
                        Pose3D botpose = result.getBotpose();
                        aimTimer.startTime();
                        TiltControl.setPosition(.35);
                        if (RightOuttake.getVelocity() > speed + 25) {
                            RightOuttake.setVelocity(0);
                        } else if (RightOuttake.getVelocity() < speed - 25) {
                            RightOuttake.setVelocity(speed + (speed * speedx)+ 80);
                        } else {
                            RightOuttake.setVelocity(speed);
                        }

                        if (LeftOuttake.getVelocity() > speed + 25) {
                            LeftOuttake.setVelocity(0);
                        } else if (LeftOuttake.getVelocity() < speed - 25) {
                            LeftOuttake.setVelocity(speed + (speed * speedx) + 80);
                        } else {
                            LeftOuttake.setVelocity(speed);
                        }

                    }
                    BottomRampServo.setPower(0);
                    BottomRampServo2.setPower(0);
                    helper3.setPower(0);
                    RightOuttake.setPower(0);
                    LeftOuttake.setPower(0);
                    Pusher.setPosition(0.47);
                    follower.followPath(paths.Path2);

                    pathState++;
                    break;

                case 2:
                    follower.followPath(paths.Path3);
                    pathState++;
                    break;

                case 3:
                    follower.followPath(paths.Path4);
                    TiltControl.setPosition(0.36);
                    pathState++;
                    break;

                case 4:
                    follower.followPath(paths.Path5);
                    pathState++;
                    break;

                case 5:

                    //RightOuttake.setVelocity(-880);
                    //LeftOuttake.setVelocity(-880);
                    RightOuttake.setVelocity(outtakespeed);
                    LeftOuttake.setVelocity(outtakespeed);
                    sleep(1000);
                    follower.followPath(paths.Path6);

                    pathState++;
                    break;

                case 6:

                    follower.followPath(paths.Path7);
                    Pusher.setPosition(0.1);
                    try {
                        Thread.sleep(50);
                    } catch(InterruptedException e) {
                        telemetry.addData("Warning","Sleeping interrupted:");
                    }
                    aimTimer.reset();
                    BottomRampServo.setPower(-1);
                    BottomRampServo2.setPower(-1);
                    helper3.setPower(1);
                    align = false;
                    follower.pausePathFollowing();
                    while(!align) {
                        LLResult result = limelight.getLatestResult();
                        double tx = result.getTx();   // Limelight angle error

                        // ---- TUNING VALUES ----
                        double kP = 0.02;             // proportional gain
                        double minPower = 0.08;       // minimum turn power
                        double maxPower = 0.30;       // max turn power
                        double deadband = 0.1;        // degrees allowed error

                        if (Math.abs(tx) > deadband) {

                            double turnPower = tx * kP;

                            // Clamp to max power
                            turnPower = Math.max(-maxPower, Math.min(maxPower, turnPower));

                            // Enforce minimum power
                            if (Math.abs(turnPower) < minPower) {
                                turnPower = Math.signum(turnPower) * minPower;
                            }

                            // Apply turn
                            leftFront.setPower(turnPower);
                            leftBack.setPower(turnPower);
                            rightFront.setPower(-turnPower);
                            rightBack.setPower(-turnPower);

                        } else {
                            // Aligned
                            leftFront.setPower(0);
                            leftBack.setPower(0);
                            rightFront.setPower(0);
                            rightBack.setPower(0);
                            align = true;
                        }
                    }
                    follower.resumePathFollowing();
                    while (aimTimer.milliseconds() < 2100) {
                        LLResult result = limelight.getLatestResult();
                        distance = distancem(result.getTa());
                        speedx = (-0.0000182763 * (distance * distance)) + (0.003602 * distance) - 0.0113504;
                        //speed = (4.4 * distance) + 740;
                        speed = (0.0061376 * (distance * distance)) + (2.66667 * distance) + 850.7619;
                        Pose3D botpose = result.getBotpose();
                        aimTimer.startTime();
                        TiltControl.setPosition(.35);
                        if (RightOuttake.getVelocity() > speed + 25) {
                            RightOuttake.setVelocity(0);
                        } else if (RightOuttake.getVelocity() < speed - 25) {
                            RightOuttake.setVelocity(speed + (speed * speedx)+ 80);
                        } else {
                            RightOuttake.setVelocity(speed);
                        }

                        if (LeftOuttake.getVelocity() > speed + 25) {
                            LeftOuttake.setVelocity(0);
                        } else if (LeftOuttake.getVelocity() < speed - 25) {
                            LeftOuttake.setVelocity(speed + (speed * speedx)+ 80);
                        } else {
                            LeftOuttake.setVelocity(speed);
                        }

                    }
                    BottomRampServo.setPower(0);
                    BottomRampServo2.setPower(0);
                    helper3.setPower(0);
                    RightOuttake.setPower(0);
                    LeftOuttake.setPower(0);
                    Pusher.setPosition(0.47);
                    pathState++;
                    break;

                case 7:

                    follower.followPath(paths.Path8);

                    pathState++;
                    break;
                case 8:
                    //RightOuttake.setVelocity(-880);
                    //LeftOuttake.setVelocity(-880);
                    RightOuttake.setVelocity(outtakespeed);
                    LeftOuttake.setVelocity(outtakespeed);
                    follower.followPath(paths.Path9);
                    pathState++;
                    break;
                case 9:
                    follower.followPath(paths.Path13);
                    pathState++;
                    break;
                case 10:
                    follower.followPath(paths.Path10);
                    Pusher.setPosition(0.1);
                    try {
                        Thread.sleep(50);
                    } catch(InterruptedException e) {
                        telemetry.addData("Warning","Sleeping interrupted:");
                    }
                    aimTimer.reset();
                    BottomRampServo.setPower(-1);
                    BottomRampServo2.setPower(-1);
                    helper3.setPower(1);
                    align = false;
                    follower.pausePathFollowing();
                    while(!align) {
                        LLResult result = limelight.getLatestResult();
                        double tx = result.getTx();   // Limelight angle error

                        // ---- TUNING VALUES ----
                        double kP = 0.02;             // proportional gain
                        double minPower = 0.08;       // minimum turn power
                        double maxPower = 0.30;       // max turn power
                        double deadband = 0.1;        // degrees allowed error

                        if (Math.abs(tx) > deadband) {

                            double turnPower = tx * kP;

                            // Clamp to max power
                            turnPower = Math.max(-maxPower, Math.min(maxPower, turnPower));

                            // Enforce minimum power
                            if (Math.abs(turnPower) < minPower) {
                                turnPower = Math.signum(turnPower) * minPower;
                            }

                            // Apply turn
                            leftFront.setPower(turnPower);
                            leftBack.setPower(turnPower);
                            rightFront.setPower(-turnPower);
                            rightBack.setPower(-turnPower);

                        } else {
                            // Aligned
                            leftFront.setPower(0);
                            leftBack.setPower(0);
                            rightFront.setPower(0);
                            rightBack.setPower(0);
                            align = true;
                        }
                    }
                    follower.resumePathFollowing();
                    while (aimTimer.milliseconds() < 2100) {
                        LLResult result = limelight.getLatestResult();
                        distance = distancem(result.getTa());
                        speedx = (-0.0000182763 * (distance * distance)) + (0.003602 * distance) - 0.0113504;
                        //speed = (4.4 * distance) + 740;
                        speed = (0.0061376 * (distance * distance)) + (2.66667 * distance) + 850.7619;
                        Pose3D botpose = result.getBotpose();
                        aimTimer.startTime();
                        TiltControl.setPosition(.35);
                        if (RightOuttake.getVelocity() > speed + 25) {
                            RightOuttake.setVelocity(0);
                        } else if (RightOuttake.getVelocity() < speed - 25) {
                            RightOuttake.setVelocity(speed + (speed * speedx)+ 80);
                        } else {
                            RightOuttake.setVelocity(speed);
                        }

                        if (LeftOuttake.getVelocity() > speed + 25) {
                            LeftOuttake.setVelocity(0);
                        } else if (LeftOuttake.getVelocity() < speed - 25) {
                            LeftOuttake.setVelocity(speed + (speed * speedx)+ 80);
                        } else {
                            LeftOuttake.setVelocity(speed);
                        }

                    }
                    BottomRampServo.setPower(0);
                    BottomRampServo2.setPower(0);
                    helper3.setPower(0);
                    RightOuttake.setPower(0);
                    LeftOuttake.setPower(0);
                    Pusher.setPosition(0.47);
                    pathState++;
                    break;
                case 11:
                    follower.followPath(paths.Path11);

                    pathState++;
                    break;
                case 12:
                    RightOuttake.setVelocity(outtakespeed);
                    LeftOuttake.setVelocity(outtakespeed);
                    //RightOuttake.setVelocity(-880);
                    //LeftOuttake.setVelocity(-880);
                    follower.followPath(paths.Path12);
                    pathState++;
                    break;
                case 13:

                    Pusher.setPosition(0.1);
                    try {
                        Thread.sleep(50);
                    } catch(InterruptedException e) {
                        telemetry.addData("Warning","Sleeping interrupted:");
                    }
                    aimTimer.reset();
                    BottomRampServo.setPower(-1);
                    BottomRampServo2.setPower(-1);

                    helper3.setPower(1);
                    align = false;
                    follower.pausePathFollowing();
                    while(!align) {
                        LLResult result = limelight.getLatestResult();
                        double tx = result.getTx();   // Limelight angle error

                        // ---- TUNING VALUES ----
                        double kP = 0.02;             // proportional gain
                        double minPower = 0.08;       // minimum turn power
                        double maxPower = 0.30;       // max turn power
                        double deadband = 0.1;        // degrees allowed error

                        if (Math.abs(tx) > deadband) {

                            double turnPower = tx * kP;

                            // Clamp to max power
                            turnPower = Math.max(-maxPower, Math.min(maxPower, turnPower));

                            // Enforce minimum power
                            if (Math.abs(turnPower) < minPower) {
                                turnPower = Math.signum(turnPower) * minPower;
                            }

                            // Apply turn
                            leftFront.setPower(turnPower);
                            leftBack.setPower(turnPower);
                            rightFront.setPower(-turnPower);
                            rightBack.setPower(-turnPower);

                        } else {
                            // Aligned
                            leftFront.setPower(0);
                            leftBack.setPower(0);
                            rightFront.setPower(0);
                            rightBack.setPower(0);
                            align = true;
                        }
                    }
                    follower.resumePathFollowing();
                    while (aimTimer.milliseconds() < 2100) {
                        LLResult result = limelight.getLatestResult();
                        distance = distancem(result.getTa());
                        speedx = (-0.0000182763 * (distance * distance)) + (0.003602 * distance) - 0.0113504;
                        //speed = (4.4 * distance) + 740;
                        speed = (0.0061376 * (distance * distance)) + (2.66667 * distance) + 850.7619;
                        Pose3D botpose = result.getBotpose();
                        aimTimer.startTime();
                        TiltControl.setPosition(.35);
                        if (RightOuttake.getVelocity() > speed + 25) {
                            RightOuttake.setVelocity(0);
                        } else if (RightOuttake.getVelocity() < speed - 25) {
                            RightOuttake.setVelocity(speed + (speed * speedx)+ 80 );
                        } else {
                            RightOuttake.setVelocity(speed);
                        }

                        if (LeftOuttake.getVelocity() > speed + 25) {
                            LeftOuttake.setVelocity(0);
                        } else if (LeftOuttake.getVelocity() < speed - 25) {

                            LeftOuttake.setVelocity(speed + (speed * speedx) + 80);
                        } else {
                            LeftOuttake.setVelocity(speed);
                        }

                    }
                    BottomRampServo.setPower(0);
                    BottomRampServo2.setPower(0);
                    helper3.setPower(0);
                    RightOuttake.setPower(0);
                    LeftOuttake.setPower(0);
                    Pusher.setPosition(0.47);
                    pathState++;
                    break;

                case 14:
                    follower.followPath(paths.Path14);
                    pathState++;
                    break;
            }
        }
        return pathState;
    }
    public double distancem(double x) {
        double AprilTagDistance = Math.pow((x/2604.88382),-0.5367);
        return AprilTagDistance;
    }
}