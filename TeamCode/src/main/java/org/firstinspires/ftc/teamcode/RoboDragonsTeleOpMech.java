package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "RoboDragonsnew2025")
public class RoboDragonsTeleOpMech extends LinearOpMode {
    private DcMotor LeftFront;
    private DcMotor LeftBack;

    private DcMotor RightFront;
    private DcMotor RightBack;

    @Override
    public void runOpMode() {
        LeftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        LeftBack = hardwareMap.get(DcMotor.class, "LeftBack");
        RightFront = hardwareMap.get(DcMotor.class, "RightFront");
        RightBack = hardwareMap.get(DcMotor.class, "RightBack");

        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                if (gamepad1.left_stick_y != 0) {
                    LeftFront.setPower(gamepad1.left_stick_y);
                    RightBack.setPower(-(gamepad1.left_stick_y));
                    RightFront.setPower(-(gamepad1.left_stick_y));
                    LeftBack.setPower((-gamepad1.left_stick_y));
                }
                if (gamepad1.right_stick_x != 0) {
                    LeftFront.setPower((gamepad1.left_stick_y));
                    RightBack.setPower(gamepad1.left_stick_y);
                    RightFront.setPower((gamepad1.left_stick_y));
                    LeftBack.setPower(gamepad1.left_stick_y);
                }
                if (gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0 && !gamepad1.right_bumper && !gamepad1.left_bumper) {
                    LeftFront.setPower(0);
                    RightBack.setPower(0);
                    RightFront.setPower(0);
                    LeftBack.setPower(0);
                }
                if (gamepad1.left_bumper) {
                    LeftFront.setPower(1);
                    RightBack.setPower(1);
                    RightFront.setPower(-1);
                    LeftBack.setPower(-1);
                }
                if (gamepad1.left_bumper) {
                    LeftFront.setPower(-1);
                    RightBack.setPower(-1);
                    RightFront.setPower(1);
                    LeftBack.setPower(1);
                }
                if (gamepad2.left_stick_y != 0) {
                    LeftFront.setPower(gamepad1.left_stick_y);
                    RightBack.setPower((gamepad1.left_stick_y));
                    RightFront.setPower(gamepad1.left_stick_y);
                    LeftBack.setPower((gamepad1.left_stick_y));
                }
                if (gamepad2.left_stick_x != 0) {
                    LeftFront.setPower((gamepad1.left_stick_y));
                    RightBack.setPower(gamepad1.left_stick_y);
                    RightFront.setPower((gamepad1.left_stick_y));
                    LeftBack.setPower(gamepad1.left_stick_y);
                }
                if (gamepad2.left_stick_x == 0 && gamepad2.left_stick_y == 0) {
                    LeftFront.setPower(0);
                    RightBack.setPower(0);
                    RightFront.setPower(0);
                    LeftBack.setPower(0);
                }
            }
        }
    }
}