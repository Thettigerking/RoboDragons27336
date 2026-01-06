package org.firstinspires.ftc.teamcode.limelight;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoTest")

public class ServoAnglePos extends OpMode {

    private Servo TiltControl;

    private double x;

    @Override
    public void init() {
        TiltControl = hardwareMap.get(Servo.class, "TiltControl");
    }

    @Override
    public void loop() {

        if(gamepad1.y){
            x++;
        }else if(gamepad1.a){
            x--;
        }

        double ServoAngle = x / 360;

        TiltControl.setPosition(ServoAngle);

        telemetry.addData("X-Value", x);

        telemetry.update();
    }
}
