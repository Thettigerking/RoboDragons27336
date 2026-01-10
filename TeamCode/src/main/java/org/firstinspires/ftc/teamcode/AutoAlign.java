package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp(name = "AutoAlign")
public class AutoAlign extends LinearOpMode {
    private DcMotorEx RightOuttake;
    private DcMotorEx LeftOuttake;
    private Servo TiltControl;

    private DcMotor Intake;
    private CRServo BottomRampServo, BottomRampServo2, helper3;
    private static final double INTAKE_POWER = -1.0;
    private static final double RAMP_POWER = -1;
    private Limelight3A limelight;
    private double distance;


    double x = 0;
    double voltage;
    @Override
    public void runOpMode() {

        Intake = hardwareMap.get(DcMotor.class, "Intake");
        BottomRampServo  = hardwareMap.get(CRServo.class, "BottomRampServo");
        BottomRampServo2 = hardwareMap.get(CRServo.class, "BottomRampServo2");
        helper3 = hardwareMap.get(CRServo.class, "helper3");

        RightOuttake = hardwareMap.get(DcMotorEx.class, "Right Motor Outtake");
        LeftOuttake = hardwareMap.get(DcMotorEx.class, "Left Motor Outtake");
        TiltControl = hardwareMap.get(Servo.class, "TiltControl");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);


        waitForStart();
        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            distance = distancem(result.getTa());

            if (gamepad2.a) {
                x = x +100;
                sleep(100);
            } else if (gamepad2.x) {
                x = x + 10;
                sleep(100);
            } else if (gamepad2.b) {
                x = x - 100;
                sleep(100);
            } else if (gamepad2.y) {
                x = x - 10;
                sleep(100);
            }
            if (gamepad2.right_trigger > 0) {
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
            TiltControl.setPosition(0.35);
            RightOuttake.setVelocity(x);
            LeftOuttake.setVelocity(-x);
            telemetry.addData("Right Power: ",RightOuttake.getPower());
            telemetry.addData("Left Power: ",LeftOuttake.getPower());
            telemetry.addData("Right RPM: ",RightOuttake.getVelocity());
            telemetry.addData("Left RPM: ",LeftOuttake.getVelocity());
            telemetry.addData("DISTANCE:",distance);

            telemetry.update();
        }
    }
    public double distancem(double x) {
        double AprilTagDistance = Math.pow((x/2604.88382),-0.5367);
        return AprilTagDistance;
    }
}