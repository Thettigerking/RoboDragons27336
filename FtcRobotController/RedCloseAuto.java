
package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous(name = "Red Far Autonomous", group = "Autonomous")
@Configurable // Panels
public class RedCloseAuto extends OpMode {
    private ElapsedTime aimTimer = new ElapsedTime();

    private double outtakespeed = -870;
    private DcMotor Intake;
    private CRServo BottomRampServo, BottomRampServo2, helper3;
    private Servo Pusher, TiltControl;
    private DcMotorEx RightOuttake, LeftOuttake;
    private Limelight3A limelight;
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56, 8, Math.toRadians(90)).mirror());

        paths = new Paths(follower); // Build paths
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        BottomRampServo  = hardwareMap.get(CRServo.class, "BottomRampServo");
        BottomRampServo2 = hardwareMap.get(CRServo.class, "BottomRampServo2");
        helper3 = hardwareMap.get(CRServo.class, "helper3");


        Pusher  = hardwareMap.get(Servo.class, "Pusher");
        TiltControl = hardwareMap.get(Servo.class, "TiltControl");

        RightOuttake = hardwareMap.get(DcMotorEx.class, "Right Motor Outtake");
        LeftOuttake  = hardwareMap.get(DcMotorEx.class, "Left Motor Outtake");

        LeftOuttake.setDirection(DcMotorSimple.Direction.REVERSE);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);

        RightOuttake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftOuttake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftOuttake.setDirection(DcMotorSimple.Direction.REVERSE);
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
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


    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.000, 8.000).mirror(),

                                    new Pose(48.000, 96.000).mirror()
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(315+270))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.000, 96.000).mirror(),

                                    new Pose(56, 8).mirror()
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(315+270), Math.toRadians(90))

                    .build();
        }
    }


    public int autonomousPathUpdate() throws InterruptedException {

        if (!follower.isBusy()) {
            switch (pathState) {


                case 0:

                    TiltControl.setPosition(0.35);
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
                    while (aimTimer.milliseconds() < 1900) {
                        aimTimer.startTime();
                        if (RightOuttake.getVelocity() > outtakespeed) {
                            RightOuttake.setVelocity(-1320);
                        } else if (RightOuttake.getVelocity() < outtakespeed) {
                            RightOuttake.setVelocity(-500);
                        } else {
                            RightOuttake.setVelocity(outtakespeed);
                        }
                        if (LeftOuttake.getVelocity() > outtakespeed) {
                            LeftOuttake.setVelocity(-1320);
                        } else if (LeftOuttake.getVelocity() < outtakespeed) {
                            LeftOuttake.setVelocity(-500);
                        } else {
                            LeftOuttake.setVelocity(outtakespeed);
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
            }
        }
        return pathState;
    }


}
    