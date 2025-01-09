package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface Subsystem {
    public abstract void initialize(HardwareMap hardwareMap, Telemetry telemetry);
    public abstract void sendTelemetry(Telemetry telemetry);
    public abstract void periodic();
    public abstract void stop();
}
