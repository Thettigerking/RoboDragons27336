package org.firstinspires.ftc.teamcode;

import static java.sql.DriverManager.println;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "RoboDragonsnew2025")
public class RoboDragonsTeleOpMech extends LinearOpMode {
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor Intake;

    private CRServo BottomRampServo;

    private CRServo BottomRampServo2;
    private Servo Pusher;

    private DcMotor rightFront;

    private DcMotor rightBack;

    private Servo TiltControl;

    private DcMotor RightOuttake;
    private DcMotor LeftOuttake;
    private Servo Pusher2;

    @Override
    public void runOpMode() {

        //
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        TiltControl = hardwareMap.get(Servo.class, "TiltControl");
        BottomRampServo = hardwareMap.get(CRServo.class, "BottomRampServo");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        BottomRampServo2 = hardwareMap.get(CRServo.class, "BottomRampServo2");
        Pusher = hardwareMap.get(Servo.class, "Pusher");
        RightOuttake = hardwareMap.get(DcMotor.class, "Right Motor Outtake");
        LeftOuttake = hardwareMap.get(DcMotor.class, "Left Motor Outtake");
        Pusher2 = hardwareMap.get(Servo.class, "Pusher2");

        // Put initialization blocks here.
        int outtakestopper = 1;
        final double[] positions = {0.17, 0.9, 0.9};
        final double[] speed = {-0.43, -0.65, -0.65};
        int index = 0;
        RightOuttake.setPower(speed[index]);
        LeftOuttake.setPower(speed[index] );
        TiltControl.setPosition(positions[index]);
        telemetry.addData("Servo pos", positions[index]);
        telemetry.addData("Instruction", "Press gamepad1.a to cycle 0 -> 0.5 -> 1");
        telemetry.update();

        boolean prevA = false; // previous state of gamepad1.a for edge detection

        waitForStart();
        boolean macro1 = false;
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                double leftX;
                double leftY;
                double rightX;

                leftX = gamepad1.left_stick_x;
                leftY = gamepad1.left_stick_y;
                rightX = gamepad1.right_stick_x;

                double leftRearPower = leftY + leftX - rightX;
                double leftFrontPower = leftY - leftX - rightX;
                double rightRearPower = -leftY + leftX - rightX;
                double rightFrontPower = -leftY - leftX - rightX;

                leftFront.setPower(leftFrontPower);
                leftBack.setPower(leftRearPower);
                rightFront.setPower(rightFrontPower);
                rightBack.setPower(rightRearPower);

                if (gamepad2.a) {
                    Intake.setPower(-1);
                    BottomRampServo.setPower(-1);
                    BottomRampServo2.setPower(-1);
                } else if (gamepad2.y) {
                    Intake.setPower(1);
                    BottomRampServo.setPower(1);
                    BottomRampServo2.setPower(1);
                } else {
                    Intake.setPower(0);
                    BottomRampServo.setPower(0);
                    BottomRampServo2.setPower(0);
                }

                if (gamepad2.b) {
                    Pusher.setPosition(0.5);
                    Pusher2.setPosition(0);
                } else {
                    Pusher.setPosition(0.85);
                    Pusher2.setPosition(1);
                }
                if (gamepad2.x) {

                    BottomRampServo.setPower(-1);
                    BottomRampServo2.setPower(-1);
                    RightOuttake.setPower(1);
                    LeftOuttake.setPower(1);
                    sleep(500);
                    Pusher.setPosition(0.8);
                    sleep(500);
                    Pusher.setPosition(0.45);
                    sleep(500);
                    Pusher.setPosition(0.8);
                    sleep(500);
                    Pusher.setPosition(0.45);
                    sleep(500);
                    Pusher.setPosition(0.8);
                    sleep(500);
                    Pusher.setPosition(0.45);
                    BottomRampServo.setPower(0);
                    BottomRampServo2.setPower(0);
                    RightOuttake.setPower(0);
                    LeftOuttake.setPower(0);
                }
                boolean currA = gamepad1.right_bumper;
                if (currA && !prevA) {
                    index = (index + 1) % positions.length;
                    double newPos = positions[index];
                    TiltControl.setPosition(newPos);

                    telemetry.addData("Toggled to", newPos);
                }


                // Always show current position and index
                telemetry.addData("Index", index);
                telemetry.addData("Position", TiltControl.getPosition());
                telemetry.update();
                prevA = currA;


            }

        }
    }
}
