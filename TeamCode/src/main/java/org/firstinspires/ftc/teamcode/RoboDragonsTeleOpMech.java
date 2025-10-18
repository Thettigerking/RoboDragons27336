package org.firstinspires.ftc.teamcode;

import static java.sql.DriverManager.println;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "RoboDragonsnew2025")
public class RoboDragonsTeleOpMech extends LinearOpMode {
    private DcMotor LeftFront;
    private DcMotor LeftBack;

    private DcMotor RightFront;

    private DcMotor RightBack;

    private Servo TiltControl;

    @Override
    public void runOpMode() {

        //
        LeftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        LeftBack = hardwareMap.get(DcMotor.class, "LeftBack");
        RightFront = hardwareMap.get(DcMotor.class, "RightFront");
        RightBack = hardwareMap.get(DcMotor.class, "RightBack");
        TiltControl = hardwareMap.get(Servo.class, "TiltControl");

    /*    LeftBack.setDirection(DcMotor.Direction.REVERSE);
        RightBack.setDirection(DcMotor.Direction.REVERSE);
        RightFront.setDirection(DcMotor.Direction.REVERSE);*/



        // Put initialization blocks here.
        waitForStart();

        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                double leftX;
                double leftY;
                double rightX;

                leftX = gamepad1.left_stick_x;
                rightX = -gamepad1.left_stick_y;
                leftY = -gamepad1.right_stick_x;

                double leftRearPower = leftY + leftX - rightX;
                double leftFrontPower = leftY - leftX - rightX;
                double rightRearPower = leftY + leftX + rightX;
                double rightFrontPower = leftY - leftX + rightX;

                LeftFront.setPower(leftFrontPower);
                LeftBack.setPower(leftRearPower);
                RightFront.setPower(rightFrontPower);
                RightBack.setPower(rightRearPower);

                double stickValue = -gamepad2.left_stick_y;
                double mapped = (stickValue + 1) / 2;
                TiltControl.setPosition(mapped);
            }
        }
    }
}
