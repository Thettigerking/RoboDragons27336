package org.firstinspires.ftc.teamcode.limelight;

import static java.lang.Math.sqrt;

import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.ftc.PinpointIMU;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "LimelightTest")

public class AprilTagLimelightExample extends OpMode {

    private Limelight3A limelight;

   // private GoBildaPinpointDriver imu;

    private double distance;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); //April tag number #0 pipeline
    //   imu = hardwareMap.get(GoBildaPinpointDriver.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
    //    imu.initialize(new GoBildaPinpointDriver(re));
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
       // YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
      //  limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llResult = limelight.getLatestResult();

        if(llResult != null && llResult.isValid()){
            Pose3D botPose = llResult.getBotpose_MT2();
            distance = getDistanceFromTag(llResult.getTa());

            telemetry.addData("distance", distance);
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Target Area", llResult.getTa());
            telemetry.addData("Botpose", botPose.toString());

        }
    }

    public double getDistanceFromTag(double ta){
        double scale = 1654.074;
        double distance = 70 / Math.sqrt(ta);
        return distance;
    }
}
