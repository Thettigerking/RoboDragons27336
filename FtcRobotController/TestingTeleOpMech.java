package org.firstinspires.ftc.teamcode;

import static java.sql.DriverManager.println;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TestingMech")
public class TestingTeleOpMech extends LinearOpMode {
    private DcMotor LeftFront;
    private DcMotor LeftBack;

    private DcMotor RightFront;
    private DcMotor RightBack;

    private DcMotor Intake;

    private CRServo BottomRampServo;

    private CRServo BottomRampServo2;

    private Servo TiltControl;

    private Servo Pusher;

    @Override
    public void runOpMode() {

        //
        BottomRampServo = hardwareMap.get(CRServo.class, "BottomRampServo");

        BottomRampServo2 = hardwareMap.get(CRServo.class, "BottomRampServo2");

        TiltControl = hardwareMap.get(Servo.class, "TiltControl");

        Pusher = hardwareMap.get(Servo.class, "Pusher");

        Intake = hardwareMap.get(DcMotor.class, "Intake");

        LeftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        LeftBack = hardwareMap.get(DcMotor.class, "LeftBack");
        RightFront = hardwareMap.get(DcMotor.class, "RightFront");
        RightBack = hardwareMap.get(DcMotor.class, "RightBack");

        LeftBack.setDirection(DcMotor.Direction.REVERSE);
        RightBack.setDirection(DcMotor.Direction.REVERSE);
        RightFront.setDirection(DcMotor.Direction.REVERSE);


        // Put initialization blocks here.
        waitForStart();

        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {

                if (gamepad1.left_stick_y != 0){
                    drive();
                }

                if (gamepad1.left_stick_x != 0){
                    turn();
                }

                if (gamepad1.right_bumper) {
                    strafeLeft();
                }

                if (gamepad1.left_bumper) {
                    strafeRight();
                }
                if (!gamepad1.right_bumper && !gamepad1.left_bumper && gamepad1.left_stick_x == 0) {
                    LeftFront.setPower(0);
                    RightBack.setPower(0);
                    RightFront.setPower(0);
                    LeftBack.setPower(0);
                }
                if (gamepad1.a){
                    intake();
                    BottomRampServo.setPower(-1);
                    BottomRampServo2.setPower(-1);
                }else{
                    nointake();
                    BottomRampServo.setPower(0);
                    BottomRampServo2.setPower(0);
                }
                if (gamepad1.dpad_up){
                    Pusher.setPosition(1);
                }
                if (gamepad1.dpad_down){
                    Pusher.setPosition(1);
                }
            }
        }
    }

    private void strafeLeft() {
        LeftFront.setPower(-1);
        RightBack.setPower(-1);
        RightFront.setPower(1);
        LeftBack.setPower(1);
    }

    private void strafeRight() {
        LeftFront.setPower(1);
        RightBack.setPower(1);
        RightFront.setPower(-1);
        LeftBack.setPower(-1);
    }

    private void turn() {
        LeftFront.setPower(-(gamepad1.left_stick_x));
        RightBack.setPower((gamepad1.left_stick_x));
        RightFront.setPower((gamepad1.left_stick_x));
        LeftBack.setPower(-(gamepad1.left_stick_x));
    }

    private void drive() {
        LeftFront.setPower(gamepad1.left_stick_y);
        RightBack.setPower(gamepad1.left_stick_y);
        RightFront.setPower(gamepad1.left_stick_y);
        LeftBack.setPower(gamepad1.left_stick_y);
    }

    private void intake() {
        Intake.setPower(-1);
    }
    private void nointake() {
        Intake.setPower(0);
    }
}
