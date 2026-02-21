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

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.concurrent.ExecutorService;

@TeleOp(name = "Shooting While Moving")
public class RoboMain2 extends LinearOpMode {
    double speed = 0;
    double ips = 0;
    double filteredIPS = 0;

    double aimOffset;
    double rawDistancea;
    double rx;
    double x;
    double y;
    private IMU imu;
    private DcMotorEx leftFront, leftBack, rightFront, rightBack;
    private DcMotorEx Intake;
    private CRServo BottomRampServo, BottomRampServo2, helper3;
    private Servo Pusher, TiltControl;
    private DcMotorEx RightOuttake, LeftOuttake;
    private Limelight3A limelight;
    private ElapsedTime myTimer = new ElapsedTime();

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
    private static final double INTAKE_POWER = -1;
    private static final double RAMP_POWER = -1;
    private static final double PRECISION_DRIVE_SCALE = 0.45; // when left trigger pressed
    private static final double DEFAULT_DRIVE_SCALE = 1.0;// --- Debounce / toggle helpers ---
    private boolean prevRightBumper = false;
    private double distance;
    private double speedx = 0;
    String shooter = "a";
    private int tiltIndex = 0;
    boolean intakeActive;
    boolean pusherExtended;
    private ElapsedTime spinupTimer = new ElapsedTime();
    private ElapsedTime shootTimer = new ElapsedTime();

    private boolean shooting = false;
    private boolean intakeEnabled = false;
    private boolean pusherOut = false;
    private boolean isDistanceTrackerActive = false ;

    class DistanceTrackerThread implements Runnable {
        public void run() {
            isDistanceTrackerActive = true ;

            while (opModeIsActive()) {
                try {
                    processDistance();

                } catch (Exception ex) {
                    // add telemetry about error
                    System.err.println("Error in distance tracker thread: " + ex.toString());
                }
            }  // while

            isDistanceTrackerActive = false ;
        }

    }

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);


        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack  = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront= hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        imu = hardwareMap.get(IMU.class,"imu");

        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        BottomRampServo  = hardwareMap.get(CRServo.class, "BottomRampServo");
        BottomRampServo2 = hardwareMap.get(CRServo.class, "BottomRampServo2");
        helper3 = hardwareMap.get(CRServo.class, "helper3");

        Pusher  = hardwareMap.get(Servo.class, "Pusher");
        TiltControl = hardwareMap.get(Servo.class, "TiltControl");

        RightOuttake = hardwareMap.get(DcMotorEx.class, "Right Motor Outtake");
        LeftOuttake  = hardwareMap.get(DcMotorEx.class, "Left Motor Outtake");
        boolean timerStarted = false;
        boolean timerStarteda = false;

// --- Variables for derivative control ---

        //

        //

        //

        //
        double lastTx = 0;
        double lastTime = 0;
        double kP = 0.02;        // proportional gain, increase for faster correction
        double kD = 0.025;       // derivative gain, optional but helps momentum
        double minPower = 0.08;  // minimum turn power
        double maxPower = 0.3;   // max turn power for faster aiming
        double deadband = 1.0;
        //
         //
         //
         //
         //
         //
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        RightOuttake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftOuttake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftOuttake.setDirection(DcMotorSimple.Direction.REVERSE);
        double p = 15.6;
        double i = 0;
        double d = 0.8;
        double f = 14.4;

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
        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        Thread dtrackerThread = new Thread(new DistanceTrackerThread()) ;

        waitForStart();

        try {
            dtrackerThread.start();
        } catch(Exception ex) {
            System.err.println("Error while starting distance tracker threadL: " + ex) ;
        }

        while (opModeIsActive()) {

            // --- Drive inputs ---
            y = gamepad1.left_stick_y;       // forward/back
            x = -gamepad1.left_stick_x;      // strafe
            double driveScale = (gamepad1.left_trigger > 0.1) ? PRECISION_DRIVE_SCALE : DEFAULT_DRIVE_SCALE;

            // --- Limelight aiming ---
            LLResult result = limelight.getLatestResult();
            double tx = 0;
            if (result != null && result.isValid()) {
                tx = result.getTx() - aimOffset;  // adjust for manual offset
            }

            double currentTime = getRuntime();
            double deltaTime = currentTime - lastTime;
            double derivative = 0;
            if (deltaTime > 0) {
                derivative = (tx - lastTx) / deltaTime;
            }

            // PID control (P + D)
            double turnPower = 0;
            if (Math.abs(tx) > deadband) {
                turnPower = kP * tx;

                // Clamp power
                turnPower = Math.max(-maxPower, Math.min(maxPower, turnPower));

                // Enforce minimum power
                if (Math.abs(turnPower) < minPower) {
                    turnPower = Math.signum(turnPower) * minPower;
                }
            }
            if (gamepad2.right_trigger > 0 || gamepad2.left_trigger > 0) {
                rx = turnPower;       // rotation command
                lastTx = tx;
                lastTime = currentTime;
                if (!timerStarted) {
                    myTimer.reset();
                    timerStarted = true;
                }
                macro = true;
                if (myTimer.milliseconds() >= 500) {
                    macroa = true;
                }


            } else {
                rx = gamepad1.right_stick_x;
                macro = false;
                macroa = false;
                timerStarted = false;
            }


            // --- Mecanum drive calculation (rotation independent of translation) ---
            double denominator = Math.max(1.0, Math.abs(y) + Math.abs(x));
            double lf = (y + x) / denominator * driveScale + rx;
            double lb = (y - x) / denominator * driveScale + rx;
            double rf = (y - x) / denominator * driveScale - rx;
            double rb = (y + x) / denominator * driveScale - rx;

            leftFront.setPower(lf);
            leftBack.setPower(lb);
            rightFront.setPower(rf);
            rightBack.setPower(rb);



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
                outtake();

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

            boolean rbb = gamepad2.left_bumper;

            if (rbb && !prevRightBumper) {  // bumper JUST pressed
                tiltIndex = (tiltIndex + 1) % TILT_POSITIONS.length;
                //TiltControl.setPosition(TILT_POSITIONS[tiltIndex]);
            }

            prevRightBumper = rbb;  // update state
            if (gamepad2.dpad_down || gamepad2.right_trigger > 0 || gamepad2.left_trigger > 0 || gamepad2.b) {
                outtake();
            } else if (gamepad2.dpad_up) {
                RightOuttake.setVelocity(6000);
                LeftOuttake.setVelocity(6000);
            } else {
                RightOuttake.setVelocity(0);
                LeftOuttake.setVelocity(0);
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
            telemetry.addData("Raw Dist", rawDistancea);
            telemetry.addData("Final Dist", distance);
            telemetry.addData("Robot IPS", ips);
            telemetry.update();
        }
    }
    final double[] OUTTAKE_POWERS = {(-0.57), (-0.45), (-0.35)};

    private void outtake() {
        //if (RightOuttake.getVelocity() > speed+25) {
        //    RightOuttake.setPower(0.2);
        //} else if (RightOuttake.getVelocity() < speed-25) {
        //    RightOuttake.setPower(.75);
        //} else {
        double voltage = 0;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            voltage = sensor.getVoltage();
        }
        double targetVelocity = 11.5/voltage;
            double p = 15.6;
            double i = 0;
            double d = 0.8;
            double f = 14.4;
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

            limelight.pipelineSwitch(0);
            if (distance < 90) {
                TiltControl.setPosition(0.35);
                RightOuttake.setVelocity((speed)*targetVelocity);
                LeftOuttake.setVelocity((speed)*targetVelocity);
            } else {
                TiltControl.setPosition(0.4);
                RightOuttake.setVelocity((speed-65)*targetVelocity);
                LeftOuttake.setVelocity((speed-65)*targetVelocity);
            }



        /*if (RightOuttake.getVelocity() > speed) {
            RightOuttake.setVelocity(distance * 8.3);
        } else if (RightOuttake.getVelocity() < speed) {
            RightOuttake.setVelocity(distance * 21.6);
        } else {
            RightOuttake.setVelocity(speed);
        }
        if (LeftOuttake.getVelocity() > speed) {
            LeftOuttake.setVelocity(distance * 8.3);
        } else if (LeftOuttake.getVelocity() < speed) {
            LeftOuttake.setVelocity(distance * 21.6);
        } else {
            LeftOuttake.setVelocity(speed);
        }*/


        //}
        //if (LeftOuttake.getVelocity() > speed+25) {
        //    LeftOuttake.setPower(0.2);
        //} else if (LeftOuttake.getVelocity() < speed-25) {
        //    LeftOuttake.setPower(.75);
        //} else {
        //LeftOuttake.setVelocity(speed);
        //}

/*                if (RightOuttake.getVelocity() > speed+25) {
            RightOuttake.setPower(0);

        } else if (RightOuttake.getVelocity() < speed-25){
            RightOuttake.setPower(-.75);
        } else {
            RightOuttake.setPower(-speed/2800);
        }
        if (LeftOuttake.getVelocity() > speed+25) {
            LeftOuttake.setPower(0);
        } else if (LeftOuttake.getVelocity() < speed-25){
            LeftOuttake.setPower(-.75);
        } else {
            LeftOuttake.setPower(speed/2800);
        }*/
    }
    public double distancem(double x) {
        double AprilTagDistance = Math.pow((x/2604.88382),-0.5367);
        return AprilTagDistance;
    }

    private void processDistance() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return;

        // Raw distance from Limelight area
        double rawDistance = distancem(result.getTa());  // inches

        // Get mecanum forward velocity
        double lf = leftFront.getVelocity();
        double rf = rightFront.getVelocity();
        double lb = leftBack.getVelocity();
        double rb = rightBack.getVelocity();

        double forwardTicksPerSec = (lf + rf + lb + rb) / 4.0;

        // Convert to inches/sec
        double inchesPerTick = 12.86 / 537.7;   // goBILDA 104mm mecanum
        double rawIPS = forwardTicksPerSec * inchesPerTick;

// low-pass filter (smooths mecanum noise)
        filteredIPS = filteredIPS * 0.8 + rawIPS * 0.2;

        double forwardSpeedIPS = filteredIPS;

        // Limelight total latency
        double totalLatencyMs =
                result.getCaptureLatency()
                        + result.getTargetingLatency()
                        + result.getParseLatency()
                        + 50;    // USB + SDK delay


        // Ball flight time
        double noteVelocity = 18.0;   // FTC flywheel ≈ 18–22 m/s
        double distanceMeters = rawDistance * 0.0254;
        double flightTime = distanceMeters / noteVelocity;
        double latencySeconds = totalLatencyMs / 1000.0;

        // Motion compensation
        double totalTime = latencySeconds + flightTime;
        double motionOffset = forwardSpeedIPS * totalTime;

        // Final corrected distance
        distance = rawDistance - motionOffset;

        // Shooter model
        speed = (0.0061376 * (distance * distance))
                + (2.66667 * distance)
                + 800.7619;
        ips = forwardSpeedIPS;
        rawDistancea = rawDistance;
    }


}
