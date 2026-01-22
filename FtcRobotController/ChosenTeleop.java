package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "TheChosenTeleop2")
public class ChosenTeleop extends LinearOpMode {

    // ================= HARDWARE =================
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private DcMotor Intake;
    private CRServo BottomRampServo, BottomRampServo2, helper3;
    private Servo Pusher, TiltControl;
    private DcMotorEx RightOuttake, LeftOuttake;
    private Limelight3A limelight;

    // ================= STATE =================
    private ElapsedTime aimTimer = new ElapsedTime();
    private boolean timerStarted = false;
    private boolean macroShoot = false;

    private double distance;
    private double speed;
    private double speedx;

    // ================= CONSTANTS =================
    private static final double INTAKE_POWER = -1.0;
    private static final double RAMP_POWER = -1.0;

    private static final double PUSHER_OPEN = 0.47;
    private static final double PUSHER_HALF = 0.10;

    private static final double PRECISION_DRIVE_SCALE = 0.45;
    private static final double DEFAULT_DRIVE_SCALE = 1.0;

    private static final double CLOSE_RANGE_CM = 120;

    private static final double TX_KP = 0.02;
    private static final double POSE_KP = 0.8;
    private static final double MAX_TURN = 0.4;

    // MEASURE THIS ON YOUR ROBOT
    private static final double SHOOTER_OFFSET_DEG = 3.0;

    private static final double[] TILT_POSITIONS = {0.45, 0.25, 0.20};
    private int tiltIndex = 0;

    @Override
    public void runOpMode() {

        // ================= INIT =================
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack  = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront= hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        Intake = hardwareMap.get(DcMotor.class, "Intake");
        BottomRampServo  = hardwareMap.get(CRServo.class, "BottomRampServo");
        BottomRampServo2 = hardwareMap.get(CRServo.class, "BottomRampServo2");
        helper3 = hardwareMap.get(CRServo.class, "helper3");

        Pusher  = hardwareMap.get(Servo.class, "Pusher");
        TiltControl = hardwareMap.get(Servo.class, "TiltControl");

        RightOuttake = hardwareMap.get(DcMotorEx.class, "Right Motor Outtake");
        LeftOuttake  = hardwareMap.get(DcMotorEx.class, "Left Motor Outtake");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        LeftOuttake.setDirection(DcMotorSimple.Direction.REVERSE);
        RightOuttake.setDirection(DcMotorSimple.Direction.REVERSE);

        RightOuttake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftOuttake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RightOuttake.setVelocityPIDFCoefficients(8, 0, 0.6, 13.5);
        LeftOuttake.setVelocityPIDFCoefficients(8, 0, 0.6, 13.5);

        TiltControl.setPosition(TILT_POSITIONS[tiltIndex]);
        Pusher.setPosition(PUSHER_OPEN);

        limelight.pipelineSwitch(0);
        limelight.start();

        waitForStart();

        // ================= LOOP =================
        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();
            Pose3D botpose = result.getBotpose();

            distance = distancem(result.getTa());

            speedx = (-0.0000182763 * distance * distance)
                    + (0.003602 * distance) - 0.0113504;

            speed = (0.0061376 * distance * distance)
                    + (2.66667 * distance) + 800.7619;

            // ================= DRIVE =================
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double driveScale = (gamepad1.left_trigger > 0.1)
                    ? PRECISION_DRIVE_SCALE
                    : DEFAULT_DRIVE_SCALE;

            double denom = Math.max(1.0, Math.abs(y) + Math.abs(x) + Math.abs(rx));

            leftFront.setPower((y + x + rx) / denom * driveScale);
            leftBack.setPower((y - x + rx) / denom * driveScale);
            rightFront.setPower((y - x - rx) / denom * driveScale);
            rightBack.setPower((y + x - rx) / denom * driveScale);

            // ================= AUTO AIM =================
            if (gamepad2.right_trigger > 0 && result.getTa() > 0) {

                if (!timerStarted) {
                    aimTimer.reset();
                    timerStarted = true;
                }

                double turn;

                if (distance > CLOSE_RANGE_CM) {
                    // FAR RANGE -> Tx
                    turn = Range.clip(result.getTx() * TX_KP, -MAX_TURN, MAX_TURN);
                } else {
                    // CLOSE RANGE -> botpose (.x / .y)
                    double xPose = botpose.getPosition().x;
                    double yPose = botpose.getPosition().y;

                    double angleToGoal = Math.atan2(yPose, xPose);
                    double offset = Math.toRadians(SHOOTER_OFFSET_DEG);

                    double error = angleToGoal - offset;
                    turn = Range.clip(error * POSE_KP, -MAX_TURN, MAX_TURN);
                }

                leftFront.setPower(turn);
                leftBack.setPower(turn);
                rightFront.setPower(-turn);
                rightBack.setPower(-turn);

                if (aimTimer.milliseconds() > 400) {
                    macroShoot = true;
                }

            } else {
                timerStarted = false;
                macroShoot = false;
            }

            // ================= INTAKE =================
            if (gamepad2.a) {
                Intake.setPower(INTAKE_POWER);
                BottomRampServo.setPower(RAMP_POWER);
                BottomRampServo2.setPower(RAMP_POWER);
                helper3.setPower(-RAMP_POWER);
            } else {
                Intake.setPower(0);
                BottomRampServo.setPower(0);
                BottomRampServo2.setPower(0);
                helper3.setPower(0);
            }

            // ================= SHOOTER =================
            if (gamepad2.dpad_down || macroShoot) {
                outtake();
            } else {
                RightOuttake.setVelocity(0);
                LeftOuttake.setVelocity(0);
            }

            // ================= PUSHER =================
            if (gamepad2.b || macroShoot) {
                Pusher.setPosition(PUSHER_HALF);
            } else {
                Pusher.setPosition(PUSHER_OPEN);
            }
            TiltControl.setPosition(0.35);

            telemetry.addData("Distance", distance);
            telemetry.addData("RPM", speed);
            telemetry.addData("Tx", result.getTx());
            telemetry.update();
        }
    }

    // ================= METHODS =================
    private void outtake() {
        RightOuttake.setVelocity(speed + (speed * speedx));
        LeftOuttake.setVelocity(speed + (speed * speedx));
    }

    public double distancem(double ta) {
        return Math.pow((ta / 2604.88382), -0.5367);
    }
}
