
package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.RoboMain;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Shooting;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Blue Far Autonomous", group = "Autonomous")
@Configurable // Panels
public class BlueCloseAuto extends LinearOpMode {
    private double outtakespeed = -890;
    public boolean autorunning = false;

    double speed = 0;
    private Pose posevalue;
    private double headingvalue;
    private DcMotor Intake;
    private ElapsedTime myTimera = new ElapsedTime();
    boolean timerStarteda = false;


    private CRServo BottomRampServo, BottomRampServo2, helper3;
    private boolean align = false;
    private double distance;
    private double speedx = 0;
    private Servo Pusher, TiltControl;
    private DcMotorEx RightOuttake, LeftOuttake;
    private Limelight3A limelight;
    private DcMotor leftFront, leftBack, rightFront, rightBack;

    private ElapsedTime aimTimer = new ElapsedTime();

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    //131

    @Override
    public void runOpMode() throws InterruptedException {

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(8.5, 8.5, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        Intake = hardwareMap.get(DcMotor.class, "Intake");
        BottomRampServo = hardwareMap.get(CRServo.class, "BottomRampServo");
        BottomRampServo2 = hardwareMap.get(CRServo.class, "BottomRampServo2");
        helper3 = hardwareMap.get(CRServo.class, "helper3");

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        Pusher = hardwareMap.get(Servo.class, "Pusher");
        TiltControl = hardwareMap.get(Servo.class, "TiltControl");

        RightOuttake = hardwareMap.get(DcMotorEx.class, "Right Motor Outtake");
        LeftOuttake = hardwareMap.get(DcMotorEx.class, "Left Motor Outtake");

        RightOuttake.setDirection(DcMotorSimple.Direction.FORWARD);
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
        while (opModeInInit()) {
            follower.updatePose();
            panelsTelemetry.debug("X", follower.getPose().getX());
            panelsTelemetry.debug("Y", follower.getPose().getY());
            panelsTelemetry.debug("Heading", follower.getPose().getHeading());
            panelsTelemetry.update(telemetry);
        }
        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            distance = distancem(result.getTa());
            double speedx = (-0.0000182763 * (distance * distance)) + (0.003602 * distance) - 0.0113504;
            double speed = (0.0061376 * (distance * distance)) + (2.66667 * distance) + 800.7619;
            LeftOuttake.setDirection(DcMotor.Direction.FORWARD);
            RightOuttake.setDirection(DcMotor.Direction.REVERSE);
            TiltControl.setPosition(0.4);
            follower.update();
            autonomousPathUpdate();
            Shooting mainteleop = new Shooting();
            if (align) {
                double tx = result.getTx();   // Limelight angle error

                // ---- TUNING VALUES ----
                double kP = 0.02;             // proportional gain
                double minPower = 0.08;       // minimum turn power
                double maxPower = 0.30;       // max turn power
                double deadband = 0.5;        // degrees allowed error

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
                    Pusher.setPosition(0.1);
                    BottomRampServo.setPower(-0.5);
                    BottomRampServo2.setPower(-0.5);
                    helper3.setPower(0.5);
                    Intake.setPower(-0.5);
                }
            }
            double voltage = 0;
            for (VoltageSensor sensor : hardwareMap.voltageSensor) {
                voltage = sensor.getVoltage();
            }
            double targetVelocity = 12.5/voltage;

            RightOuttake.setVelocity(((mainteleop.outtake(speedx,speed,"REDFAR",RightOuttake.getVelocity()))-20) * targetVelocity);
            LeftOuttake.setVelocity(((mainteleop.outtakeleft(speedx,speed,"REDFAR",LeftOuttake.getVelocity()))-20)  * targetVelocity);
            // Log values to Panels and Driver Station
            panelsTelemetry.debug("Path State", pathState);
            panelsTelemetry.debug("X", follower.getPose().getX());
            panelsTelemetry.debug("Y", follower.getPose().getY());
            panelsTelemetry.debug("Heading", follower.getPose().getHeading());
            panelsTelemetry.debug("RightOuttake:", RightOuttake.getVelocity());
            panelsTelemetry.debug("LeftOuttake", LeftOuttake.getVelocity());
            panelsTelemetry.debug("Warning:", "Threshold not found! This may cause issues!");
            panelsTelemetry.update(telemetry);

        }
    }

    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;


        public Paths(Follower follower) {
            Pose posevalue = follower.getPose();
            Double headingvalue = follower.getHeading();
            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    posevalue,

                                    new Pose(56.000, 15.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(250), Math.toRadians(300))

                    .build();
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.000, 15.000)
                                    ,

                                    new Pose(56.000, 20.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(headingvalue), Math.toRadians(300))

                    .build();
        }
    }


    public void autonomousPathUpdate() {
        if (!follower.isBusy()) {
            switch (pathState) {
                case 0:
                    Pusher.setPosition(0.47);
                    follower.followPath(paths.Path2);
                    pathState = 1;
                    break;
                case 1:
                    follower.pausePathFollowing();
                    align = true;

                    pathState = 2;
                case 2:
            }
        }
    }


    public double distancem(double x) {
        double AprilTagDistance = Math.pow((x / 2604.88382), -0.5367);
        return AprilTagDistance;
    }

}