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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red Autonomous", group = "Autonomous")
@Configurable // Panels
public class RedAuto extends LinearOpMode {

    private double outtakespeed = -890;
    private DcMotor Intake;
    private CRServo BottomRampServo, BottomRampServo2, helper3;
    private Servo Pusher, TiltControl;
    private DcMotorEx RightOuttake, LeftOuttake;
    private Limelight3A limelight;
    private boolean align = false;
    private DcMotor leftFront, leftBack, rightFront, rightBack;

    private ElapsedTime aimTimer = new ElapsedTime();

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    //131
//131
    @Override
    public void runOpMode() throws InterruptedException {

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(8.5, 8.5, Math.toRadians(90)).mirror());

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

        limelight.start();
        RightOuttake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftOuttake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftOuttake.setDirection(DcMotorSimple.Direction.REVERSE);
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        waitForStart();
        while(opModeInInit()){
            follower.updatePose();
        }
        while(opModeIsActive()) {


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
            panelsTelemetry.update(telemetry);

        }
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

        public PathChain Path14;

        public PathChain Path15;
        public Paths(Follower follower) {
            initPaths(follower);
        }

        public void initPaths(Follower follower) {
            Pose posevalue = follower.getPose();
            Double headingvalue = follower.getHeading();
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(posevalue, new Pose(48.000, 106).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(headingvalue), Math.toRadians(317-90))
                    .build();
            Path15 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(48.000, 106).mirror(), new Pose(48.000, 96).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(317-90), Math.toRadians(317-90))
                    .build();
            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(48.000, 96.000).mirror(), new Pose(48.000, 85.000-2).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(317-90), Math.toRadians(180+180))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(48.000, 85.000-2).mirror(), new Pose(15, 85.000-2).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180+180), Math.toRadians(180+180))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(15, 85.000-2).mirror(), new Pose(35.000, 85.000-2).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180+180), Math.toRadians(90+180))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(35.000, 85.000-2).mirror(), new Pose(14.000, 77.000-2).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90+180), Math.toRadians(90+180))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(14.000, 77.000-2).mirror(), new Pose(48.000, 96.000-2).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90+180), Math.toRadians(314-90))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(48.000, 96.000-2).mirror(), new Pose(54.000, 65-2).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(314-90), Math.toRadians(180+180))
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(54.000, 63-2).mirror(), new Pose(8.000, 60-2).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180+180), Math.toRadians(180+180))
                    .build();
            Path9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(8, 60-2).mirror(), new Pose(20, 63-2).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180+180), Math.toRadians(180+180))
                    .build();

            Path13 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(20, 63-2).mirror(), new Pose(52, 92-2).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180+180), Math.toRadians(306-90))
                    .build();

            Path10 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(52, 94-2).mirror(), new Pose(41.000, 40.500-2).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(306-90), Math.toRadians(180+180))
                    .build();

            Path11 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(41.000, 38.500-2).mirror(), new Pose(8, 38.500-2).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180+180), Math.toRadians(180+180))
                    .build();

            Path12 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(8, 38.500-2).mirror(), new Pose(48.000, 96.000-2).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180+180), Math.toRadians(312-90))
                    .build();

            Path14 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(52, 92-2).mirror(), new Pose(24, 88.000-2).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(312-90), Math.toRadians(90+180))
                    .build();

        }
    }

    public int autonomousPathUpdate() throws InterruptedException {
        double voltage = 0;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            voltage = sensor.getVoltage();
        }
        double targetVelocity = 12/voltage;
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

                    pathState = 1;
                    break;
                case 1:
                    follower.followPath(paths.Path15);

                    pathState = 2;
                    break;
                case 2:
                    align = false;
                    Pusher.setPosition(0.1);
                    try {
                        Thread.sleep(50);
                    } catch(InterruptedException e) {
                        telemetry.addData("Warning","Sleeping interrupted:");
                    }
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
                    aimTimer.reset();
                    while (aimTimer.milliseconds() < 1900) {
                        aimTimer.startTime();
                        if (RightOuttake.getVelocity() > outtakespeed) {
                            RightOuttake.setVelocity(-1300 * targetVelocity);
                        } else if (RightOuttake.getVelocity() < outtakespeed) {
                            RightOuttake.setVelocity(0);
                        } else {
                            RightOuttake.setVelocity(outtakespeed * targetVelocity);
                        }
                        if (LeftOuttake.getVelocity() > outtakespeed) {
                            LeftOuttake.setVelocity(-1300 * targetVelocity);
                        } else if (LeftOuttake.getVelocity() < outtakespeed) {
                            LeftOuttake.setVelocity(0);
                        } else {
                            LeftOuttake.setVelocity(outtakespeed * targetVelocity);
                        }

                    }
                    BottomRampServo.setPower(0);
                    BottomRampServo2.setPower(0);
                    helper3.setPower(0);
                    RightOuttake.setPower(0);
                    LeftOuttake.setPower(0);
                    Pusher.setPosition(0.47);
                    follower.followPath(paths.Path2);

                    pathState = 3;
                    break;

                case 3:
                    follower.followPath(paths.Path3);
                    pathState = 4;
                    break;

                case 4:
                    follower.followPath(paths.Path4);
                    TiltControl.setPosition(0.36);
                    pathState = 5;
                    break;

                case 5:
                    follower.followPath(paths.Path5);
                    pathState = 6;
                    break;

                case 6:

                    //RightOuttake.setVelocity(-880);
                    //LeftOuttake.setVelocity(-880);
                    RightOuttake.setVelocity(outtakespeed);
                    LeftOuttake.setVelocity(outtakespeed);
                    sleep(200);
                    follower.followPath(paths.Path6);

                    pathState = 7;
                    break;

                case 7:
                    align = false;
                    follower.followPath(paths.Path7);
                    Pusher.setPosition(0.1);
                    try {
                        Thread.sleep(50);
                    } catch(InterruptedException e) {
                        telemetry.addData("Warning","Sleeping interrupted:");
                    }
                    BottomRampServo.setPower(-1);
                    BottomRampServo2.setPower(-1);
                    helper3.setPower(1);
                    aimTimer.reset();
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
                    while (aimTimer.milliseconds() < 1900) {
                        aimTimer.startTime();
                        if (RightOuttake.getVelocity() > outtakespeed) {
                            RightOuttake.setVelocity(-1120 * targetVelocity);
                        } else if (RightOuttake.getVelocity() < outtakespeed) {
                            RightOuttake.setVelocity(-520 * targetVelocity);
                        } else {
                            RightOuttake.setVelocity((outtakespeed+100) * targetVelocity);
                        }
                        if (LeftOuttake.getVelocity() > outtakespeed) {
                            LeftOuttake.setVelocity(-1120 * targetVelocity);
                        } else if (LeftOuttake.getVelocity() < outtakespeed) {
                            LeftOuttake.setVelocity(-550 * targetVelocity);
                        } else {
                            LeftOuttake.setVelocity((outtakespeed+100) * targetVelocity);
                        }

                    }
                    BottomRampServo.setPower(0);
                    BottomRampServo2.setPower(0);
                    helper3.setPower(0);
                    RightOuttake.setPower(0);
                    LeftOuttake.setPower(0);
                    Pusher.setPosition(0.47);
                    pathState = 8;
                    break;

                case 8:

                    follower.followPath(paths.Path8);

                    pathState = 9;
                    break;
                case 9:
                    //RightOuttake.setVelocity(-880);
                    //LeftOuttake.setVelocity(-880);
                    RightOuttake.setVelocity(outtakespeed);
                    LeftOuttake.setVelocity(outtakespeed);
                    follower.followPath(paths.Path9);
                    pathState = 10;
                    break;
                case 10:
                    follower.followPath(paths.Path13);
                    pathState = 11;
                    break;
                case 11:
                    follower.followPath(paths.Path10);
                    Pusher.setPosition(0.1);
                    try {
                        Thread.sleep(50);
                    } catch(InterruptedException e) {
                        telemetry.addData("Warning","Sleeping interrupted:");
                    }
                    BottomRampServo.setPower(-1);
                    BottomRampServo2.setPower(-1);
                    helper3.setPower(1);
                    align = false;
                    follower.pausePathFollowing();
                    aimTimer.reset();
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
                    aimTimer.reset();
                    while (aimTimer.milliseconds() < 1900) {
                        aimTimer.startTime();
                        if (RightOuttake.getVelocity() > outtakespeed) {
                            RightOuttake.setVelocity(-1090 * targetVelocity);
                        } else if (RightOuttake.getVelocity() < outtakespeed) {
                            RightOuttake.setVelocity(-550 * targetVelocity);
                        } else {
                            RightOuttake.setVelocity((outtakespeed-50) * targetVelocity);
                        }
                        if (LeftOuttake.getVelocity() > outtakespeed) {
                            LeftOuttake.setVelocity(-1090 * targetVelocity);
                        } else if (LeftOuttake.getVelocity() < outtakespeed) {
                            LeftOuttake.setVelocity(-550 * targetVelocity);
                        } else {
                            LeftOuttake.setVelocity((outtakespeed-50) * targetVelocity);
                        }

                    }
                    BottomRampServo.setPower(0);
                    BottomRampServo2.setPower(0);
                    helper3.setPower(0);
                    RightOuttake.setPower(0);
                    LeftOuttake.setPower(0);
                    Pusher.setPosition(0.47);
                    pathState = 12;
                    break;
                case 12:
                    follower.followPath(paths.Path11);

                    pathState = 13;
                    break;
                case 13:
                    RightOuttake.setVelocity(outtakespeed);
                    LeftOuttake.setVelocity(outtakespeed);
                    //RightOuttake.setVelocity(-880);
                    //LeftOuttake.setVelocity(-880);
                    follower.followPath(paths.Path12);
                    pathState = 14;
                    break;
                case 14:

                    Pusher.setPosition(0.1);
                    try {
                        Thread.sleep(50);
                    } catch(InterruptedException e) {
                        telemetry.addData("Warning","Sleeping interrupted:");
                    }
                    BottomRampServo.setPower(-1);
                    BottomRampServo2.setPower(-1);
                    helper3.setPower(1);
                    aimTimer.reset();
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
                    while (aimTimer.milliseconds() < 1900) {
                        aimTimer.startTime();
                        if (RightOuttake.getVelocity() > outtakespeed) {
                            RightOuttake.setVelocity(-1130 * targetVelocity);
                        } else if (RightOuttake.getVelocity() < outtakespeed) {
                            RightOuttake.setVelocity(-550 * targetVelocity);
                        } else {
                            RightOuttake.setVelocity(outtakespeed * targetVelocity);
                        }
                        if (LeftOuttake.getVelocity() > outtakespeed) {
                            LeftOuttake.setVelocity(-1130 * targetVelocity);
                        } else if (LeftOuttake.getVelocity() < outtakespeed) {
                            LeftOuttake.setVelocity(-550 * targetVelocity);
                        } else {
                            LeftOuttake.setVelocity(outtakespeed * targetVelocity);
                        }

                    }
                    BottomRampServo.setPower(0);
                    BottomRampServo2.setPower(0);
                    helper3.setPower(0);
                    RightOuttake.setPower(0);
                    LeftOuttake.setPower(0);
                    Pusher.setPosition(0.47);
                    pathState = 15;
                    break;

                case 15:
                    follower.followPath(paths.Path14);
                    pathState = 16;
                    break;
            }
        }
        return pathState;
    }

}