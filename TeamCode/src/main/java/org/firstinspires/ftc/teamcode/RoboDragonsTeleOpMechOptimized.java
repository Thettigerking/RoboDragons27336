package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "RoboDragonsnew2025_Optimized")
public class RoboDragonsTeleOpMechOptimized extends LinearOpMode {

    // --- Hardware ---
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private DcMotor Intake;
    private CRServo BottomRampServo, BottomRampServo2, helperservo, helperservo2, helper3;
    private Servo Pusher, Pusher2, TiltControl;
    private DcMotorEx RightOuttake, LeftOuttake;


    // --- Constants / Tuning (easy to change) ---
    private static boolean pushervar = false;
    //0.2, 0.3
    private static final double[] TILT_POSITIONS = {0.45, 0.25, 0.2};
    private static final double[] speeds = {1200, 1040, 770};
    private static final double[] speedssmall = {1150, 1000, 730};
    private static final double PUSHER_OPEN = 0.47;
    private static final double PUSHER_HALF = 0.1;
    private static final double PUSHER_CLOSE = 0.45;
    private static final double INTAKE_POWER = -1.0;
    private static final double RAMP_POWER = -1.0;
    private static final double PRECISION_DRIVE_SCALE = 0.45; // when left trigger pressed
    private static final double DEFAULT_DRIVE_SCALE = 1.0;// --- Debounce / toggle helpers ---
    private boolean prevRightBumper = false;

    String shooter = "a";
     private int tiltIndex = 0;
//a
    @Override
    public void runOpMode() {

        // Hardware mapping
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack  = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront= hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        Intake = hardwareMap.get(DcMotor.class, "Intake");
        BottomRampServo  = hardwareMap.get(CRServo.class, "BottomRampServo");
        BottomRampServo2 = hardwareMap.get(CRServo.class, "BottomRampServo2");
        helperservo = hardwareMap.get(CRServo.class, "helperservo");
        helperservo2 = hardwareMap.get(CRServo.class, "helperservo2");
        helper3 = hardwareMap.get(CRServo.class, "helper3");

        Pusher  = hardwareMap.get(Servo.class, "Pusher");
        Pusher2 = hardwareMap.get(Servo.class, "Pusher2");
        TiltControl = hardwareMap.get(Servo.class, "TiltControl");

        RightOuttake = hardwareMap.get(DcMotorEx.class, "Right Motor Outtake");
        LeftOuttake  = hardwareMap.get(DcMotorEx.class, "Left Motor Outtake");

        // Motor directions - set this to match how motors are physically mounted
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        // Behavior when power = 0
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Ensure motors won't try to use encoders unless you want them to
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        RightOuttake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftOuttake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initial positions / powers
        tiltIndex = 0;
        TiltControl.setPosition(TILT_POSITIONS[tiltIndex]);
        //RightOuttake.setPower(OUTTAKE_POWERS[tiltIndex]);
        //LeftOuttake.setPower(OUTTAKE_POWERS[tiltIndex]);
        Pusher.setPosition(PUSHER_OPEN);
        Pusher2.setPosition(1.0);

        telemetry.addData(" ", "Ready to start");
        telemetry.update();
        waitForStart();

        // Main loop
        while (opModeIsActive()) {
            final double[] OUTTAKE_POWERS = {(-0.57), (-0.45), (-0.35)};
            // ----- Drive (mecanum) -----
            // Standard, normalized mecanum drive
            double y = gamepad1.left_stick_y; // forward
            double x = -gamepad1.left_stick_x;  // strafe
            double rx = gamepad1.right_stick_x; // rotation

            // precision mode when left trigger pressed
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

            // ----- Intake & Ramp control -----
            if (gamepad2.a) {
                Intake.setPower(INTAKE_POWER);
                BottomRampServo.setPower(RAMP_POWER);
                BottomRampServo2.setPower(RAMP_POWER);
                helperservo.setPower(RAMP_POWER);
                helper3.setPower(-RAMP_POWER);
                helperservo2.setPower(RAMP_POWER);
            } else if (gamepad2.y) {
                // example: alternate mode - change as needed
                Intake.setPower(0.6);
                BottomRampServo.setPower(0.6);
                BottomRampServo2.setPower(0.6);
                helperservo.setPower(0.6);
                helperservo2.setPower(0.6);
                helper3.setPower(-0.6);
            } else {
                Intake.setPower(0);
                BottomRampServo.setPower(0);
                BottomRampServo2.setPower(0);
                helperservo.setPower(0);
                helperservo2.setPower(0);
                helper3.setPower(0);
            }
            if (gamepad2.right_bumper) {
                if (pushervar) {
                    pushervar = false;
                    Pusher2.setPosition(0.0);
                } else if (!pushervar) {
                    pushervar = true;
                    Pusher2.setPosition(1.0);
                }
            }

            // ----- Pusher control (simple) -----
            if (gamepad2.b) {
                Pusher.setPosition(PUSHER_HALF);
                //Pusher2.setPosition(0.0);
            } else {
                Pusher.setPosition(PUSHER_OPEN);
                //Pusher2.setPosition(1.0);
            }


            // ----- Tilt toggle using gamepad1.right_bumper (debounced) -----

// ---- Tilt Toggle with Debounce ----
            boolean rbb = gamepad2.left_bumper;

            if (rbb && !prevRightBumper) {  // bumper JUST pressed
                tiltIndex = (tiltIndex + 1) % TILT_POSITIONS.length;
                TiltControl.setPosition(TILT_POSITIONS[tiltIndex]);
            }


            prevRightBumper = rbb;  // update state
            if (gamepad2.dpad_down) {
                    if (RightOuttake.getVelocity() > speeds[tiltIndex]) {
                        RightOuttake.setPower(0);

                    } else if (RightOuttake.getVelocity() < speedssmall[tiltIndex]){
                        RightOuttake.setPower(-.75);
                    } else {
                        RightOuttake.setPower(OUTTAKE_POWERS[tiltIndex]);
                    }
                    if (LeftOuttake.getVelocity() < -speeds[tiltIndex]) {
                        LeftOuttake.setPower(0);
                    } else if (LeftOuttake.getVelocity() > -speedssmall[tiltIndex]){
                        LeftOuttake.setPower(-.75);
                    } else {
                        LeftOuttake.setPower(OUTTAKE_POWERS[tiltIndex]);
                    }
                } else if (gamepad2.dpad_up) {
              RightOuttake.setPower(1);
                LeftOuttake.setPower(1);
            } else {
                RightOuttake.setPower(0);
                LeftOuttake.setPower(0);
            }
            if (tiltIndex == 0) {
                shooter = "Far";
            } else if (tiltIndex == 1) {
                shooter = "Middle";
            } else if (tiltIndex == 2) {
                shooter = "Close";
            } else {
                shooter = "FTC robot controller has corrupted! Disk: Failed. How did we get here?";
            }

            // ----- Telemetry (minimal but helpful) -----
            telemetry.addData("TiltIndex", tiltIndex);
            telemetry.addData("TiltPos", TiltControl.getPosition());
            telemetry.addData("DriveScale", driveScale);
            telemetry.addData("Shooter range:",shooter);
            telemetry.addData("Right outtake velocity:",RightOuttake.getVelocity());
            telemetry.addData("Left outtake velocity:",LeftOuttake.getVelocity());
            telemetry.update();
        }

        // Ensure everything stops when opmode ends
    }

}
