package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.Shooting;

@TeleOp(name = "SpeedChanger")
public class SpeedChanger extends LinearOpMode {
    double speed = 0;
    double aimOffset;
    private IMU imu;
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private DcMotor Intake;
    private CRServo BottomRampServo, BottomRampServo2, helper3;
    private Servo Pusher, TiltControl;
    private DcMotorEx RightOuttake, LeftOuttake;
    private Limelight3A limelight;
    private ElapsedTime myTimer = new ElapsedTime();
    int state;

    private ElapsedTime myTimera = new ElapsedTime();

    private double distancet;
    private  boolean macroa = false;

    private  boolean macroab = false;



    boolean intakeActive;
    boolean pusherExtended;
    private ElapsedTime spinupTimer = new ElapsedTime();
    private ElapsedTime shootTimer = new ElapsedTime();

    private boolean shooting = false;
    private boolean intakeEnabled = false;
    private boolean pusherOut = false;

    private static boolean pushervar = false;
    private static final double[] TILT_POSITIONS = {0.45, 0.25, 0.2};
    private static final double[] speeds = {1200, 1040, 770};
    private static final double[] speedssmall = {1150, 1000, 730};
    private static final double PUSHER_OPEN = 0.47;
    private static final double PUSHER_HALF = 0.1;
    private static final double PUSHER_CLOSE = 0.45;
    private static final double INTAKE_POWER = -1.0;
    private static final double RAMP_POWER = -1;
    private static final double PRECISION_DRIVE_SCALE = 0.45; // when left trigger pressed
    private static final double DEFAULT_DRIVE_SCALE = 1.0;// --- Debounce / toggle helpers ---
    private boolean prevRightBumper = false;
    private double distance;
    private double speedx = 0;
    String shooter = "a";
    private int tiltIndex = 0;
    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);


        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack  = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront= hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        imu = hardwareMap.get(IMU.class,"imu");

        Intake = hardwareMap.get(DcMotor.class, "Intake");
        BottomRampServo  = hardwareMap.get(CRServo.class, "BottomRampServo");
        BottomRampServo2 = hardwareMap.get(CRServo.class, "BottomRampServo2");
        helper3 = hardwareMap.get(CRServo.class, "helper3");

        Pusher  = hardwareMap.get(Servo.class, "Pusher");
        TiltControl = hardwareMap.get(Servo.class, "TiltControl");

        RightOuttake = hardwareMap.get(DcMotorEx.class, "Right Motor Outtake");
        LeftOuttake  = hardwareMap.get(DcMotorEx.class, "Left Motor Outtake");
        boolean timerStarted = false;
        boolean timerStarteda = false;



        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        RightOuttake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftOuttake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftOuttake.setDirection(DcMotorSimple.Direction.REVERSE);


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
        boolean manual = false;
//        RightOuttake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        LeftOuttake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        tiltIndex = 0;
        TiltControl.setPosition(TILT_POSITIONS[tiltIndex]);
        boolean macro = false;
        boolean macrob = false;
        double p = 16.4;
        double i = 0;
        double d = 9.8;
        double f = 13.8;
        double speedsubtract = 0.98;
        Pusher.setPosition(PUSHER_OPEN);
        limelight.start();
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);

        telemetry.addData(" ", "Ready to start");
        telemetry.update();
        LeftOuttake.setDirection(DcMotor.Direction.FORWARD);
        RightOuttake.setDirection(DcMotor.Direction.REVERSE);
        LeftOuttake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RightOuttake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        TiltControl.setPosition(.4);

        waitForStart();

        while (opModeIsActive()) {
            double[] decreasetype = {0.01, 0.1, 1};
            //if (gamepad1.start) {
             //   state = state + 1;
              //  if (state > 3) {
                    state = 1;
               // }
            //}
            LLResult result = limelight.getLatestResult();
            distance = distancem(result.getTa());
            speedx = (-0.0000182763 * (distance * distance)) + (0.003602 * distance) - 0.0113504;
            //speed = (4.4 * distance) + 740;
            speed = (0.0061376 * (distance * distance)) + (2.66667 * distance) + 800.7619;
            RightOuttake.setVelocityPIDFCoefficients(
                    p,   // P
                    i,    // I
                    d,    // D
                    f    // F (THIS MATTERS)
            );

            LeftOuttake.setVelocityPIDFCoefficients(
                    p,
                    i,
                    d,
                    f
            );
            if (gamepad1.bWasReleased()) {
                p = p + decreasetype[state];
            }
            if (gamepad1.xWasReleased()) {
                p = p - decreasetype[state];
            }
            if (gamepad1.yWasReleased()) {
                i = i + decreasetype[state];
            }
            if (gamepad1.aWasReleased()) {
                i = i - decreasetype[state];
            }
            if (gamepad1.dpadUpWasReleased()) {
                d = d + decreasetype[state];
            }
            if (gamepad1.dpadDownWasReleased()) {
                d = d - decreasetype[state];
            }
            if (gamepad1.leftBumperWasReleased()) {
                f = f + decreasetype[state];
            }
            if (gamepad1.rightBumperWasReleased()) {
                f = f - decreasetype[state];
            }
            if (gamepad1.dpadLeftWasReleased()) {
                speedsubtract = speedsubtract + 0.01;
            }
            if (gamepad1.dpadRightWasReleased()) {
                speedsubtract = speedsubtract - 0.01;
            }

            if (gamepad2.right_trigger > 0 && distance < 500 || gamepad2.b && distance < 500) {

                macro = true;

                // Start timer ONCE
                if (!timerStarted) {
                    myTimer.reset();
                    timerStarted = true;
                }

                // =========================
                // MANUAL OVERRIDE
                // =========================
                if (gamepad2.right_bumper || gamepad1.right_bumper) {
                    manual = gamepad2.a;
                } else {

                    // =========================
                    // AUTO ALIGN (P CONTROL)
                    // =========================
                    if (gamepad2.right_trigger > 0) {
                        aimOffset = 8;   // degrees (positive = right, negative = left)
                    } else if (gamepad2.left_trigger > 0) {
                        aimOffset = 2;   // degrees (positive = right, negative = left)

                    }
                    double tx = result.getTx() - aimOffset;   // Limelight angle error

                    // ---- TUNING VALUES ----
                    double kP = 0.02;             // proportional gain
                    double minPower = 0.08;       // minimum turn power
                    double maxPower = 0.30;       // max turn power
                    double deadband = 0.005;    // degrees allowed error

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
                    }
                }

                // =========================
                // DELAY BEFORE INTAKE
                // =========================
                if (myTimer.milliseconds() >= 500) {
                    macroa = true;
                }

            } else {
                // =========================
                // RESET WHEN TRIGGER RELEASED
                // =========================
                macro = false;
                macroa = false;
                timerStarted = false;

            }

            boolean shootButton =
                    gamepad2.right_trigger > 0 ||
                            gamepad2.left_trigger > 0 ||
                            gamepad2.b;

            if (shootButton) {

                // First press
                if (!shooting) {
                    shooting = true;
                    intakeEnabled = false;
                    spinupTimer.reset();
                    shootTimer.reset();
                }

                // Spin flywheel
                // Enable intake AFTER 500ms
                if (!intakeEnabled && spinupTimer.milliseconds() > 500) {
                    intakeEnabled = true;
                }

                // Run intake once enabled
                if (intakeEnabled) {
                    Intake.setPower(INTAKE_POWER);
                    BottomRampServo.setPower(RAMP_POWER);
                    BottomRampServo2.setPower(RAMP_POWER);
                    helper3.setPower(-RAMP_POWER);
                }

                // Rapid fire pusher
                if (intakeEnabled &&
                        Math.abs(RightOuttake.getVelocity() - speed) < 40) {

                    if (!pusherOut) {
                        Pusher.setPosition(PUSHER_HALF);
                        shootTimer.reset();
                        pusherOut = true;
                    }

                    if (pusherOut && shootTimer.milliseconds() > 120) {
                        Pusher.setPosition(PUSHER_OPEN);
                    }

                    if (shootTimer.milliseconds() > 240) {
                        pusherOut = false;
                    }
                }

            } else {

                // Stop shooter
                shooting = false;
                intakeEnabled = false;
                pusherOut = false;

                Pusher.setPosition(PUSHER_OPEN);
                RightOuttake.setVelocity(0);
                LeftOuttake.setVelocity(0);

                // =============================
                // MANUAL INTAKE CONTROL
                // =============================
                if (gamepad2.a) {
                    Intake.setPower(INTAKE_POWER);
                    BottomRampServo.setPower(RAMP_POWER);
                    BottomRampServo2.setPower(RAMP_POWER);
                    helper3.setPower(-RAMP_POWER);

                } else if (gamepad2.y) {
                    Intake.setPower(-INTAKE_POWER);
                    BottomRampServo.setPower(-RAMP_POWER);
                    BottomRampServo2.setPower(-RAMP_POWER);
                    helper3.setPower(RAMP_POWER);

                } else {
                    Intake.setPower(0);
                    BottomRampServo.setPower(0);
                    BottomRampServo2.setPower(0);
                    helper3.setPower(0);
                }
            }
            if (gamepad2.right_bumper) {
                if (pushervar) {
                    pushervar = false;
                } else if (!pushervar) {
                    pushervar = true;
                }
            }
            if (macro || macrob) {
                Pusher.setPosition(PUSHER_HALF);
            } else {
                Pusher.setPosition(PUSHER_OPEN);
            }

            LeftOuttake.setVelocity(speed*speedsubtract);
            RightOuttake.setVelocity(speed*speedsubtract);
            telemetry.addData("P: ",p);
            telemetry.addData("I: ",i);
            telemetry.addData("D: ",d);
            telemetry.addData("F: ",f);
            telemetry.addData("speedsubtract:",speedsubtract);
            telemetry.update();

        }
    }
    public double distancem(double x) {
        double AprilTagDistance = Math.pow((x/2604.88382),-0.5367);
        return AprilTagDistance;
    }
}

