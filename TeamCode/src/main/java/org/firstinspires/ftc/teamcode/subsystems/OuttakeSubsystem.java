package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;

public class OuttakeSubsystem implements Subsystem {

    private final MotorGroup motors;

    public OuttakeSubsystem(MotorGroup motors) {
        this.motors = motors;
    }

    public void setPower(double power) {
        motors.setPower(power);
    }

    public void stop() {
        motors.setPower(0);
    }
}
