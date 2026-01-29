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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.Objects;

@TeleOp(name = "TheChosenTeleop")
public class RoboMain extends LinearOpMode {
    double speed = 0;
    double rightvelocity;
    double leftvelocity;
    String shootingtype = "NONE";
    double x;
    double y;
    double rx;
    private IMU imu;
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private DcMotor Intake;
    private CRServo BottomRampServo, BottomRampServo2, helper3;
    private Servo Pusher, TiltControl;
    private DcMotorEx RightOuttake, LeftOuttake;
    private Limelight3A limelight;
    private ElapsedTime myTimer = new ElapsedTime();

    boolean btake = false;
    boolean ltake = false;
    boolean rtake = false;

    private ElapsedTime myTimera = new ElapsedTime();

    private double distancet;
    private  boolean macroa = false;

    private  boolean macroab = false;





    private static boolean pushervar = false;
    private static final double[] TILT_POSITIONS = {0.45, 0.25, 0.2};
    private static final double[] speeds = {1200, 1040, 770};
    private static final double[] speedssmall = {1150, 1000, 730};
    private static final double PUSHER_OPEN = 0.47;
    private static final double PUSHER_HALF = 0.1;
    private static final double PUSHER_CLOSE = 0.45;
    private static final double INTAKE_POWER = -1.0;
    private static final double RAMP_POWER = -1;
    boolean outtakeready = false;
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

        limelight.pipelineSwitch(0);

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

        Pusher.setPosition(PUSHER_OPEN);
        limelight.start();
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);

        telemetry.addData(" ", "Ready to start");
        telemetry.update();
        LeftOuttake.setDirection(DcMotor.Direction.FORWARD);
        RightOuttake.setDirection(DcMotor.Direction.REVERSE);
        TiltControl.setPosition(.35);

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            distance = distancem(result.getTa());
            double speedx = (-0.0000182763 * (distance * distance)) + (0.003602 * distance) - 0.0113504;
            //speed = (4.4 * distance) + 740;
            double speed = (0.0061376 * (distance * distance)) + (2.66667 * distance) + 800.7619;
            BlueCloseAuto farauto = new BlueCloseAuto();
            Pose3D botpose = result.getBotpose();
            final double[] OUTTAKE_POWERS = {(-0.57), (-0.45), (-0.35)};
            // ALIGNING {
            if (gamepad2.right_trigger > 0 && distance < 500 || gamepad2.b && distance < 500) {

                macro = true;

                // Start timer ONCE
                if (!timerStarted) {
                    myTimer.reset();
                    timerStarted = true;
                }
                if (gamepad2.right_bumper || gamepad1.right_bumper) {
                    manual = gamepad2.a;
                } else {
                    double tx = result.getTx();   // Limelight angle error

                    // ---- TUNING VALUES ----
                    double kP = 0.02;             // proportional gain
                    double minPower = 0.08;       // minimum turn power
                    double maxPower = 0.30;       // max turn power
                    double deadband = 0.5;        // degrees allowed error

                    if (Math.abs(tx) > deadband) {
                        rx = tx * kP;
                    } else {
                        rx = 0;
                    }
                }

                // }
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
                rx = gamepad1.right_stick_x;;

            }
            //DRIVING {
             y = gamepad1.left_stick_y; // forward
             x = -gamepad1.left_stick_x;  // strafe

            double driveScale = (gamepad1.left_trigger > 0.1) ? PRECISION_DRIVE_SCALE : DEFAULT_DRIVE_SCALE;

            double denominator = Math.max(1.0, Math.abs(y) + Math.abs(x) + Math.abs(rx));
            double lf = (y + x + rx) / denominator * driveScale;
            double lb = (y - x + rx) / denominator * driveScale;
            double rf = (y - x - rx) / denominator * driveScale;
            double rb = (y + x - rx) / denominator * driveScale;

            leftFront.setPower(lf);
            leftBack.setPower(lb);
            rightFront.setPower(rf);
            rightBack.setPower(rb);

            // }
            if (gamepad2.a || macroa || macrob ) {
                if (gamepad2.right_bumper || gamepad1.right_bumper) {
                    if (manual) {
                        Intake.setPower(INTAKE_POWER);
                        BottomRampServo.setPower(RAMP_POWER);
                        BottomRampServo2.setPower(RAMP_POWER);
                        helper3.setPower(-RAMP_POWER);
                    }
                } else {
                    Intake.setPower(INTAKE_POWER);
                    BottomRampServo.setPower(RAMP_POWER);
                    BottomRampServo2.setPower(RAMP_POWER);
                    helper3.setPower(-RAMP_POWER);
                }
            } else if (gamepad2.y) {
                Intake.setPower(0.6);
                BottomRampServo.setPower(0.6);
                BottomRampServo2.setPower(0.6);
                helper3.setPower(-0.6);
            } else {
                Intake.setPower(0);
                BottomRampServo.setPower(0);
                BottomRampServo2.setPower(0);
                helper3.setPower(0);
            }
            if (gamepad2.right_bumper) {
                if (pushervar) {
                    pushervar = false;
                } else if (!pushervar) {
                    pushervar = true;
                }
            }


            if (gamepad2.left_trigger > 0  && distance < 500) {

                macrob = true;
                // Start timer ONCE


                // Drive alignment
                if (result.getTx() >= 2.8) {
                    leftFront.setPower(0.14);
                    leftBack.setPower(0.14);
                    rightBack.setPower(-0.14);
                    rightFront.setPower(-0.14);
                } else if (result.getTx() <= 1.6) {
                    leftFront.setPower(-0.14);
                    leftBack.setPower(-0.14);
                    rightBack.setPower(0.14);
                    rightFront.setPower(0.14);
                } else {
                    leftFront.setPower(0);
                    leftBack.setPower(0);
                    rightBack.setPower(0);
                    rightFront.setPower(0);
                    if (!timerStarteda) {
                        myTimera.reset();
                        timerStarteda = true;
                    }
                    if (myTimer.milliseconds() >= 2000) {
                        if (outtakeready) {
                            macroab = true;
                        } else {
                            macroab = false;
                        }
                    }

                }




            } else {
                // Reset everything when X is released
                macrob = false;
                macroab = false;
                timerStarteda = false;
            }


            if ( macro || macrob) {
                Pusher.setPosition(PUSHER_HALF);
            } else {
                Pusher.setPosition(PUSHER_OPEN);
            }

            boolean rbb = gamepad2.left_bumper;

            if (rbb && !prevRightBumper) {  // bumper JUST pressed
                tiltIndex = (tiltIndex + 1) % TILT_POSITIONS.length;
                //TiltControl.setPosition(TILT_POSITIONS[tiltIndex]);
            }

            prevRightBumper = rbb;  // update state
            if (gamepad2.right_trigger > 0)  {
                rtake = true;
                RightOuttake.setVelocity(outtake(speedx,speed,"CLOSE",RightOuttake.getVelocity()));
                LeftOuttake.setVelocity(outtakeleft(speedx,speed,"CLOSE",LeftOuttake.getVelocity()));
                TiltControl.setPosition(.35);

            } else if (gamepad2.left_trigger > 0 ) {
                ltake = true;
                RightOuttake.setVelocity(outtake(speedx,speed,"BLUEFAR",RightOuttake.getVelocity()));
                LeftOuttake.setVelocity(outtakeleft(speedx,speed,"BLUEFAR",LeftOuttake.getVelocity()));

                TiltControl.setPosition(.4);

            }
            else if (gamepad2.dpad_up) {
                RightOuttake.setVelocity(6000);
                LeftOuttake.setVelocity(6000);
            }  else if (gamepad2.b) {
                RightOuttake.setVelocity(outtake(speedx,speed,"REDFAR",RightOuttake.getVelocity()));
                LeftOuttake.setVelocity(outtakeleft(speedx,speed,"REDFAR",LeftOuttake.getVelocity()));

                TiltControl.setPosition(.4);

            } else {
                RightOuttake.setVelocity(0);
                LeftOuttake.setVelocity(0);
                shootingtype = "NONE";
            }
            if (tiltIndex == 0) {
                shooter = "Far";
            } else if (tiltIndex == 1) {
                shooter = "Middle";
            } else if (tiltIndex == 2) {
                shooter = "Close";
            }
            // First, tell Limelight which way your robot is facing
            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
            double heading = angles.getYaw(BNO055IMU.AngleUnit.DEGREES.toAngleUnit());

            double robotYaw = heading;
            limelight.updateRobotOrientation(robotYaw);
            if (result != null && result.isValid()) {
                Pose3D botpose_mt2 = result.getBotpose_MT2();
                if (botpose_mt2 != null) {
                    double xa = botpose_mt2.getPosition().x;
                    double ya = botpose_mt2.getPosition().y;
                    telemetry.addData("MT2 Location:", "(" + xa + ", " + ya + ")");
                }
            }

            //            Pose2d pose = imu.getPose();
            //distance = 13/Math.tan(result.getTx());
            telemetry.addData("DEBUG:","");
            telemetry.addData("DISTANCE:",distance);
            telemetry.addData("SPEED:",speed);

            //         telemetry.addData("X (IMU)", pose.position.x);
            //       telemetry.addData("Y (IMU)", pose.position.y);
            //     telemetry.addData("Heading (deg,IMU)", Math.toDegrees(pose.heading.toDouble()));
            telemetry.addData("DriveScale", driveScale);
            telemetry.addData("Right outtake velocity:",RightOuttake.getVelocity());
            telemetry.addData("Left outtake velocity:",LeftOuttake.getVelocity());
            telemetry.addData("Speedx",speedx);
            telemetry.addData("tx:",result.getTx());
            telemetry.addData("Orentation",botpose.getOrientation());
            telemetry.addData("Pitch",botpose.getOrientation().getPitch());
            telemetry.addData("yaw",botpose.getOrientation().getYaw());
            telemetry.addData("roll",botpose.getOrientation().getRoll());

            telemetry.update();
        }
    }
    final double[] OUTTAKE_POWERS = {(-0.57), (-0.45), (-0.35)};

    public double outtake(double sspeedx, double sspeed, String shoottype, double RightCurrVeloc) {
        BlueCloseAuto farauto = new BlueCloseAuto();
         if (Objects.equals(shoottype, "BLUEFAR")) {
            if (RightCurrVeloc > sspeed + 100) {
                return (100);

            } else if (RightCurrVeloc < sspeed - 100) {
                return (sspeed + 275);

            } else {
                return (sspeed - 375);
            }


        }
        else if (Objects.equals(shoottype, "CLOSE")) {
            if (RightCurrVeloc > sspeed + 25) {
                return (0);
            } else if (RightCurrVeloc < sspeed - 25) {
                return (sspeed + (sspeed * sspeedx));
            } else {
                return (sspeed);
            }
        }
            else if (Objects.equals(shoottype, "REDFAR")){
                if (RightCurrVeloc > sspeed + 100) {
                    return (100);

                } else if (RightCurrVeloc < sspeed - 100) {
                    return (sspeed + 275);

                } else {
                    return (sspeed - 375);
                }

            }
        return (0);
    }
    public double outtakeleft(double sspeedx, double sspeed, String shoottype, double LeftCurrVeloc) {
        BlueCloseAuto farauto = new BlueCloseAuto();
        if (Objects.equals(shoottype, "BLUEFAR")) {
            if (LeftCurrVeloc > sspeed + 100) {
                return (100);

            } else if (LeftCurrVeloc < sspeed - 100) {
                return (sspeed + 275);

            } else {
                return (sspeed - 375);
            }
        }
        else if (Objects.equals(shoottype, "CLOSE")) {
            if (LeftCurrVeloc > sspeed + 25) {
                return (0);
            } else if (LeftCurrVeloc < sspeed - 25) {
                return (sspeed + (sspeed * sspeedx));
            } else {

                return (sspeed);

            }
        }
        else if (Objects.equals(shoottype, "REDFAR")){
            if (LeftCurrVeloc > sspeed + 100) {
                return (100);

            } else if (LeftCurrVeloc < sspeed - 100) {
                return (sspeed + 275);

            } else {
                return (sspeed - 375);

            }
        }
        return (0);
    }
    public double distancem(double x) {
        double AprilTagDistance = Math.pow((x/2604.88382),-0.5367);
        return AprilTagDistance;
    }
}

