package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name = "AprilTagTest")
public class AprilTagWebcamExample extends OpMode {

    private Servo TiltControl;
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);
        TiltControl = hardwareMap.get(Servo.class, "TiltControl");
    }

    @Override
    public void loop() {
        aprilTagWebcam.update();

        AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificId(20);
        aprilTagWebcam.displayDetectionTelemetry(id20);
        telemetry.addData("Angle:", aprilTagWebcam.angleOgOutake20());
        TiltControl.setPosition(aprilTagWebcam.angleOgOutake20() / 300.0);

        AprilTagDetection id24 = aprilTagWebcam.getTagBySpecificId(24);
        aprilTagWebcam.displayDetectionTelemetry(id24);

        telemetry.update();
    }
}

//something
