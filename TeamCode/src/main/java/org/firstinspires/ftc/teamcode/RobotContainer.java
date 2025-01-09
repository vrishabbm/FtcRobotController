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
    static Pivot pivot = Pivot.getInstance();
    static Viper viper = Viper.getInstance();

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

    public static void periodic() {
        for(Subsystem subsystem: subsystems) {
            subsystem.periodic();
        }
    }

    public static void stop() {
        for(Subsystem subsystem: subsystems) {
            subsystem.stop();
        }
    }

    public static Action reachHighBasket() {
        return new SequentialAction(
                pivot.pivotArm(60),
                pivot.stopPivot(),
                viper.moveViper(25),
                viper.stopViper()
        );
    }

    public static Action zeroPivotAndViper() {
        return new SequentialAction(
                viper.moveViper(0),
                viper.stopViper(),
                pivot.pivotArm(0),
                pivot.stopPivot()
        );
    }
}
