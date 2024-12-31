package org.firstinspires.ftc.teamcode.hardware.robots;

import org.firstinspires.ftc.teamcode.hardware.Motors;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class RobotHardwareSuperDwaraka extends RobotHardware {
    private static RobotHardwareSuperDwaraka instance;
    public RobotHardwareSuperDwaraka() {
        pivotP = 0.1;
        pivotI = 0.0;
        pivotD = 0.0004;
        pivotGearRatio = 4.0/1.0;
        pivotMotor = Motors.GoBilda_5203_312RPM;
        pivotTicksPerRev = pivotMotor.getEncoderResolution() / pivotGearRatio;

        viperP = 0.1;
        viperI = 0.0;
        viperD = 0.0004;
        viperGearRatio = 1.0/1.0;
        viperMotor = Motors.GoBilda_5203_312RPM;
        viperTicksPerRev = viperMotor.getEncoderResolution() / viperGearRatio;
        viperPulleyDiameter = 2; //inches
        viperInchesPerTick = viperPulleyDiameter/ viperTicksPerRev;
    }

    public static RobotHardwareSuperDwaraka getInstance() {
        if (instance == null) {
            instance = new RobotHardwareSuperDwaraka();
        }

        return instance;
    }
}
