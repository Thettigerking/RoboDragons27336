package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Mechanisms.ColorSensorDetection;

@TeleOp(name = "ColorSensorTest")

public class ColorSensorTest extends OpMode {
    ColorSensorDetection bench = new ColorSensorDetection();
    ColorSensorDetection.DetectedColor detectedColor;
    private Limelight3A limelight3A;
    private DcMotor Intake;
    private CRServo BottomRampServo, BottomRampServo2, helper3;
    private double INTAKE_POWER = -1.0;
    private double RAMP_POWER = -1.0;
    boolean P = true;
    boolean G = false;
    boolean P2 = false;
    boolean G2 = true;

    @Override
    public void init() {
        bench.init(hardwareMap);

        Intake = hardwareMap.get(DcMotor.class, "Intake");
        BottomRampServo  = hardwareMap.get(CRServo.class, "BottomRampServo");
        BottomRampServo2 = hardwareMap.get(CRServo.class, "BottomRampServo2");
        helper3 = hardwareMap.get(CRServo.class, "helper3");

        limelight3A = hardwareMap.get(Limelight3A .class, "limelight");
        limelight3A.pipelineSwitch(0);
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
        }

        if(llResult != null & llResult.isValid() & llResult.getPipelineIndex() == 22){
            if(detectedColor != null & detectedColor.equals("pruple") & P == true) {
                Intake.setPower(INTAKE_POWER);
                BottomRampServo.setPower(RAMP_POWER);
                BottomRampServo2.setPower(RAMP_POWER);
                helper3.setPower(-RAMP_POWER);

                try {
                    Thread.sleep(2000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }

                Intake.setPower(0);
                BottomRampServo.setPower(0);
                BottomRampServo2.setPower(0);
                helper3.setPower(0);

                P = false;
                G = true;
            }
            if(detectedColor != null & detectedColor.equals("green") & G == true) {
                Intake.setPower(INTAKE_POWER);
                BottomRampServo.setPower(RAMP_POWER);
                BottomRampServo2.setPower(RAMP_POWER);
                helper3.setPower(-RAMP_POWER);

                try {
                    Thread.sleep(2000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }

                Intake.setPower(0);
                BottomRampServo.setPower(0);
                BottomRampServo2.setPower(0);
                helper3.setPower(0);

               P = true;
               G = false;
            }
        } else if(llResult != null & llResult.isValid() & llResult.getPipelineIndex() == 23) {
            if(detectedColor != null & detectedColor.equals("pruple") & P == true) {
                Intake.setPower(INTAKE_POWER);
                BottomRampServo.setPower(RAMP_POWER);
                BottomRampServo2.setPower(RAMP_POWER);
                helper3.setPower(-RAMP_POWER);

                try {
                    Thread.sleep(2000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }

                Intake.setPower(0);
                BottomRampServo.setPower(0);
                BottomRampServo2.setPower(0);
                helper3.setPower(0);

               P = false;
               P2 = true;
            }
            if(detectedColor != null & detectedColor.equals("purple") & P2 == true) {
                Intake.setPower(INTAKE_POWER);
                BottomRampServo.setPower(RAMP_POWER);
                BottomRampServo2.setPower(RAMP_POWER);
                helper3.setPower(-RAMP_POWER);

                try {
                    Thread.sleep(2000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }

                Intake.setPower(0);
                BottomRampServo.setPower(0);
                BottomRampServo2.setPower(0);
                helper3.setPower(0);

                P2 = false;
                G = true;
            }
            if(detectedColor != null & detectedColor.equals("green") & G == true) {
                Intake.setPower(INTAKE_POWER);
                BottomRampServo.setPower(RAMP_POWER);
                BottomRampServo2.setPower(RAMP_POWER);
                helper3.setPower(-RAMP_POWER);

                try {
                    Thread.sleep(2000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }

                Intake.setPower(0);
                BottomRampServo.setPower(0);
                BottomRampServo2.setPower(0);
                helper3.setPower(0);

                G = false;
                P = true;
            }
        }else if(llResult != null & llResult.isValid() & llResult.getPipelineIndex() == 21) {
            if(detectedColor != null & detectedColor.equals("pruple") & G2 == true) {
                Intake.setPower(INTAKE_POWER);
                BottomRampServo.setPower(RAMP_POWER);
                BottomRampServo2.setPower(RAMP_POWER);
                helper3.setPower(-RAMP_POWER);

                try {
                    Thread.sleep(2000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }

                Intake.setPower(0);
                BottomRampServo.setPower(0);
                BottomRampServo2.setPower(0);
                helper3.setPower(0);

                G2 = false;
                G = true;
            }
            if(detectedColor != null & detectedColor.equals("purple") & G == true) {
                Intake.setPower(INTAKE_POWER);
                BottomRampServo.setPower(RAMP_POWER);
                BottomRampServo2.setPower(RAMP_POWER);
                helper3.setPower(-RAMP_POWER);

                try {
                    Thread.sleep(2000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }

                Intake.setPower(0);
                BottomRampServo.setPower(0);
                BottomRampServo2.setPower(0);
                helper3.setPower(0);

                G = false;
                P2 = true;
            }
            if(detectedColor != null & detectedColor.equals("green") & P2 == true) {
                Intake.setPower(INTAKE_POWER);
                BottomRampServo.setPower(RAMP_POWER);
                BottomRampServo2.setPower(RAMP_POWER);
                helper3.setPower(-RAMP_POWER);

                try {
                    Thread.sleep(2000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }

                Intake.setPower(0);
                BottomRampServo.setPower(0);
                BottomRampServo2.setPower(0);
                helper3.setPower(0);

                P2 = false;
                G2 = true;
            }
        }
    }
}

//something