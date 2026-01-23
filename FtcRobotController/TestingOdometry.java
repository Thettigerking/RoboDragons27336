package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "TestingOdometry")
public class TestingOdometry extends OpMode {

    GoBildaPinpointDriver odo;

    private DcMotor LeftFront;
    private DcMotor LeftBack;
    private DcMotor RightFront;
    private DcMotor RightBack;

    @Override
    public void init(){

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        LeftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        LeftBack = hardwareMap.get(DcMotor.class, "LeftBack");
        RightFront = hardwareMap.get(DcMotor.class, "RightFront");
        RightBack = hardwareMap.get(DcMotor.class, "RightBack");

        LeftBack.setDirection(DcMotor.Direction.REVERSE);
        RightBack.setDirection(DcMotor.Direction.REVERSE);
        RightFront.setDirection(DcMotor.Direction.REVERSE);

        odo.setOffsets(0,170,DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();
        Pose2D startingPosition = new Pose2D(DistanceUnit.MM, -923.925, 1604.47,AngleUnit.RADIANS, 0);odo.setPosition(startingPosition);
        odo.setPosition(startingPosition);
    }

    @Override
    public void loop() {



    }


}
