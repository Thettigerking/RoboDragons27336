package org.firstinspires.ftc.teamcode.limelight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Mechanisms.ColorSensorDetection;

@Autonomous(name = "LimelightColorTest")

public class LimelightColorTest extends OpMode {
    ColorSensorDetection bench = new ColorSensorDetection();
    ColorSensorDetection.DetectedColor detectedColor;
    private Limelight3A limelight3A;
    private DcMotor Intake;
    private CRServo BottomRampServo, BottomRampServo2, helper3;
    private double INTAKE_POWER = -1.0;
    private double RAMP_POWER = -1.0;
    boolean GPP = false;
    boolean PGP = false;
    boolean PPG = false;
    boolean P1 =false;
    boolean P2 = false;
    boolean P = true;
    boolean G1 = false;
    boolean G2 = false;
    boolean G = true;
    int id = 0;

    @Override
    public void init() {
        bench.init(hardwareMap);

        Intake = hardwareMap.get(DcMotor.class, "Intake");
        BottomRampServo  = hardwareMap.get(CRServo.class, "BottomRampServo");
        BottomRampServo2 = hardwareMap.get(CRServo.class, "BottomRampServo2");
        helper3 = hardwareMap.get(CRServo.class, "helper3");

        limelight3A = hardwareMap.get(Limelight3A .class, "limelight");
        limelight3A.pipelineSwitch(1);
    }

    @Override
    public void start() {
        limelight3A.start();
    }

    @Override
    public void loop() {
        detectedColor = bench.getDetectedColor(telemetry);
        telemetry.addData("Color Detected", detectedColor);

        LLResult llResult = limelight3A.getLatestResult();

        if(llResult != null && llResult.isValid()) {

            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Target Area", llResult.getTa());

            id = llResult.getFiducialResults().get(0).getFiducialId();
        }

        if(id == 21) {
            GPP = true;
        }else if(id == 22) {
            PGP = true;
        }else if(id == 23){
            PPG = true;
        }

        if(GPP == true) {
            telemetry.addLine("GPP");

            if(detectedColor == ColorSensorDetection.DetectedColor.GREEN & G == true){
                IntakeOn();
                try {
                    Thread.sleep(2000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                IntakeOff();
                G = false;
                P1 = true;
            }

            if(detectedColor == ColorSensorDetection.DetectedColor.PURPLE & P1 == true){
                IntakeOn();
                try {
                    Thread.sleep(3000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                IntakeOff();
                P1 = false;
                P2 = true;
            }

            if(detectedColor == ColorSensorDetection.DetectedColor.PURPLE & P2 == true){
                IntakeOn();
                try {
                    Thread.sleep(3000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                IntakeOff();
                P2 = false;
                G = true;
            }

        }else if(PGP == true) {
            telemetry.addLine("PGP");

            if(detectedColor == ColorSensorDetection.DetectedColor.PURPLE & P == true){
                IntakeOn();
                try {
                    Thread.sleep(3000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                IntakeOff();
                P = false;
                G1 = true;
            }

            if(detectedColor == ColorSensorDetection.DetectedColor.GREEN & G1 == true){
                IntakeOn();
                try {
                    Thread.sleep(3000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                IntakeOff();
                G1 = false;
                P = true;
            }

        }else if(PPG == true) {
            telemetry.addLine("PPG");

            if(detectedColor == ColorSensorDetection.DetectedColor.PURPLE & P == true){
                IntakeOn();
                try {
                    Thread.sleep(3000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                IntakeOff();
                P = false;
                P1 = true;
            }

            if(detectedColor == ColorSensorDetection.DetectedColor.PURPLE & P1 == true){
                IntakeOn();
                try {
                    Thread.sleep(3000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                IntakeOff();
                P1 = false;
                G1 = true;
            }

            if(detectedColor == ColorSensorDetection.DetectedColor.GREEN & G1 == true){
                IntakeOn();
                try {
                    Thread.sleep(3000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                IntakeOff();
                G1 = false;
                P = true;
            }
        }
    }

    private void IntakeOn() {
        Intake.setPower(INTAKE_POWER);
        BottomRampServo.setPower(RAMP_POWER);
        BottomRampServo2.setPower(RAMP_POWER);
        helper3.setPower(-RAMP_POWER);
    }

    private void IntakeOff() {
        Intake.setPower(0);
        BottomRampServo.setPower(0);
        BottomRampServo2.setPower(0);
        helper3.setPower(0);
    }
}
