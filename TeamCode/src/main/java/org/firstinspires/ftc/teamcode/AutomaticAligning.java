package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp(name = "RoboDragonsnew2025")
public class AutomaticAligning extends LinearOpMode {
    private VoltageSensor voltSensor;
    private DcMotorEx RightOuttake;
    private DcMotorEx LeftOuttake;
    double x = 0;
double voltage;
    @Override
    public void runOpMode() {


        RightOuttake = hardwareMap.get(DcMotorEx.class, "Right Motor Outtake");
        LeftOuttake = hardwareMap.get(DcMotorEx.class, "Left Motor Outtake");
        voltSensor = hardwareMap.voltageSensor.get("voltSensor");
        waitForStart();
        while (opModeIsActive()) {
            voltage = voltSensor.getVoltage()/12;
            if (gamepad1.a) {
                x = x +100;
            } else if (gamepad1.x) {
                x = x + 10;
            } else if (gamepad1.b) {
                x = x - 100;
            } else if (gamepad1.y) {
                x = x - 10;
            }
            RightOuttake.setPower(x);
            LeftOuttake.setPower(-x);
            telemetry.addData("Right Power: ",RightOuttake.getPower());
            telemetry.addData("Left Power: ",LeftOuttake.getPower());
            telemetry.addData("Right RPM: ",RightOuttake.getVelocity());
            telemetry.addData("Left RPM: ",LeftOuttake.getVelocity());
            telemetry.update();
        }
    }
}
