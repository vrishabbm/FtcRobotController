package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.Viper;

import java.util.ArrayList;

public class RobotContainer {
    public static Pivot pivot = new Pivot();
    public static Viper viper = new Viper();

    public static ArrayList<Subsystem> subsystems;

    public static void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        subsystems.add(pivot);
        subsystems.add(viper);

        for(Subsystem subsystem: subsystems) {
            subsystem.initialize(hardwareMap, telemetry);
        }
    }

    public static void sendTelemetry(Telemetry telemetry) {
        for(Subsystem subsystem: subsystems) {
            subsystem.sendTelemetry(telemetry);
        }
    }

    public static void periodic(Telemetry telemetry) {
        for(Subsystem subsystem: subsystems) {
            subsystem.periodic(telemetry);
        }
    }

    public static void stop(Telemetry telemetry) {
        for(Subsystem subsystem: subsystems) {
            subsystem.stop(telemetry);
        }
    }

    public Action reachHighBasket() {
        return new SequentialAction(
                viper.extendSlide(0),
                pivot.pivotArm(90),
                viper.extendSlide(25)
        );
    }
}
