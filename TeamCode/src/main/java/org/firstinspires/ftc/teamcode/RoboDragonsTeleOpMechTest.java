package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcontroller.external.samples.RobotAutoDriveByEncoder_Linear.DRIVE_SPEED;
import static java.sql.DriverManager.println;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "RoboDragonsnew2025")
public class RoboDragonsTeleOpMechTest extends LinearOpMode {
    private DcMotor LeftFront;
    private DcMotor LeftBack;

    private DcMotor RightFront;
    private DcMotor RightBack;

    @Override
    public void runOpMode() {
        //
        LeftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        LeftBack = hardwareMap.get(DcMotor.class, "LeftBack");
        RightFront = hardwareMap.get(DcMotor.class, "RightFront");
        RightBack = hardwareMap.get(DcMotor.class, "RightBack");

        LeftBack.setDirection(DcMotor.Direction.REVERSE);
        RightBack.setDirection(DcMotor.Direction.REVERSE);
        RightFront.setDirection(DcMotor.Direction.REVERSE);


        // Put initialization blocks here.
        waitForStart();
        while (opModeIsActive()) {

            double leftX;
            double leftY;
            double rightX;

            // Strafer Mode
            leftX = gamepad1.left_stick_x * DRIVE_SPEED;
            leftY = gamepad1.left_stick_y * DRIVE_SPEED;
            rightX = gamepad1.right_stick_x * DRIVE_SPEED;

            double leftRearPower = leftY + leftX - rightX;
            double leftFrontPower = leftY - leftX - rightX;
            double rightRearPower = leftY - leftX + rightX;
            double rightFrontPower = leftY + leftX + rightX;

            LeftFront.setPower(leftFrontPower);
            LeftBack.setPower(leftRearPower);
            RightFront.setPower(rightFrontPower);
            RightBack.setPower(rightRearPower);
        }
    }
}
