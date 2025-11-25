package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcontroller.external.samples.RobotAutoDriveByEncoder_Linear.DRIVE_SPEED;
import static java.sql.DriverManager.println;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Flywheel")
public class RoboDragonsTeleOpMechTest extends LinearOpMode {
    private DcMotor LeftFront;
    private DcMotor LeftBack;

    private DcMotor RightFront;
    private DcMotor RightBack;

    private Servo TiltControl;

//    private DcMotor Outtake1;
//    private DcMotor Outtake2;

    @Override
    public void runOpMode() {
        boolean debug_mode = true;
        LeftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        LeftBack = hardwareMap.get(DcMotor.class, "LeftBack");
        RightFront = hardwareMap.get(DcMotor.class, "RightFront");
        RightBack = hardwareMap.get(DcMotor.class, "RightBack");
        TiltControl = hardwareMap.get(Servo.class, "TiltControl");
/*        Outtake1 = hardwareMap.get(DcMotor.class, "Outtake1");
        Outtake2 = hardwareMap.get(DcMotor.class, "Outtake2");*/

/*        LeftBack.setDirection(DcMotor.Direction.REVERSE);
        RightBack.setDirection(DcMotor.Direction.REVERSE);
        RightFront.setDirection(DcMotor.Direction.REVERSE);*/
        TiltControl.setPosition(0);
        boolean lastA = false;
        boolean lastY = false;
        double TiltControlx = 0.0;
        // Put initialization blocks here.
        waitForStart();
        while (opModeIsActive()) {
        /*    double leftX;
            double leftY;
            double rightX;

            leftX = gamepad1.left_stick_x;
            leftY = gamepad1.left_stick_y;
            rightX = gamepad1.right_stick_x;

            double leftRearPower = leftY + leftX - rightX;
            double leftFrontPower = leftY - leftX - rightX;
            double rightRearPower = -leftY + leftX - rightX;
            double rightFrontPower = -leftY - leftX - rightX;

            LeftFront.setPower(leftFrontPower);
            LeftBack.setPower(leftRearPower);
            RightFront.setPower(rightFrontPower);
            RightBack.setPower(rightRearPower);*/

            double outtakePower = gamepad2.right_stick_y * 3/4;
            LeftFront.setPower(outtakePower);
            LeftBack.setPower(outtakePower);


            double stickValue = -gamepad2.left_stick_y;
        double mapped = (stickValue + 1) / 2;
        TiltControl.setPosition(mapped);

        telemetry.addData("Tilt: ", TiltControl.getPosition());
            telemetry.addData("DEBUG MODE = ", debug_mode);
            telemetry.addData("Right Trigger", gamepad2.right_trigger);
            telemetry.addData("Left Trigger", gamepad2.left_trigger);
        telemetry.update();

        }

    }

}