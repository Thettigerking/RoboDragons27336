package org.firstinspires.ftc.teamcode;

import static java.sql.DriverManager.println;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "RoboDragonsnew2025")
public class MainTeleop extends LinearOpMode {
    private DcMotor LeftFront;
    private DcMotor LeftBack;

    private DcMotor RightFront;

    private DcMotor RightBack;

    private DcMotor Outtake;

    private DcMotor Outtake2;

    private Servo TiltControl;

    @Override
    public void runOpMode() {
        // Motors
        LeftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        LeftBack = hardwareMap.get(DcMotor.class, "LeftBack");
        RightFront = hardwareMap.get(DcMotor.class, "RightFront");
        RightBack = hardwareMap.get(DcMotor.class, "RightBack");
        Outtake = hardwareMap.get(DcMotor.class, "Outtake");
        Outtake2 = hardwareMap.get(DcMotor.class, "Outtake2");

        // Servos
        TiltControl = hardwareMap.get(Servo.class, "TiltControl");


        waitForStart();

        if (opModeIsActive()) {
            TiltControl.setPosition(0);
            boolean lastA = false;
            boolean lastY = false;
            double TiltControlx = 0.0;

            // DEBUG MODE
            boolean debug_mode = false;

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

                double outtakePower = gamepad2.right_stick_y * 3/4;
                LeftFront.setPower(outtakePower);
                LeftBack.setPower(outtakePower);


                double stickValue = -gamepad2.left_stick_y;
                double mapped = (stickValue + 1) / 2;
                TiltControl.setPosition(mapped);

                telemetry.addData("DEBUG MODE = ", debug_mode);
                if (debug_mode) {
                    telemetry.addData("Tilt: ", TiltControl.getPosition());
                    telemetry.addData("Right Trigger", gamepad2.right_trigger);
                    telemetry.addData("Left Trigger", gamepad2.left_trigger);
                    telemetry.update();
                }

            }
        }
    }
}
