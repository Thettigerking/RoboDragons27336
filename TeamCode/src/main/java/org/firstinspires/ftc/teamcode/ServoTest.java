package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcontroller.external.samples.RobotAutoDriveByEncoder_Linear.DRIVE_SPEED;
import static java.sql.DriverManager.println;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoTest")
public class ServoTest extends LinearOpMode {
    private Servo Pusher;
    @Override
    public void runOpMode() {
        Pusher  = hardwareMap.get(Servo.class, "Pusher");
        // Put initialization blocks here.
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("PUSGHERRRRRKFHR", Pusher.getPosition());
            telemetry.update();

        }

    }

}