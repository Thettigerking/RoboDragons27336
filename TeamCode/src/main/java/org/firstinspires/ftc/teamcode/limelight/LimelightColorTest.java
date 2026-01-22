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
        }else if(PGP == true) {
            telemetry.addLine("PGP");
        }else if(PPG == true) {
            telemetry.addLine("PPG");
        }
    }
}
