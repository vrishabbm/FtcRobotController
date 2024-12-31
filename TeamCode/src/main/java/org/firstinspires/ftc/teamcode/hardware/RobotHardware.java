package org.firstinspires.ftc.teamcode.hardware;

import org.firstinspires.ftc.teamcode.hardware.robots.RobotHardwareSuperDwaraka;

public class RobotHardware {
    private static Bot selectedRobot = Bot.SUPER_DWARAKA; //default
    public static RobotHardware robot = RobotHardware.getInstance();

    public enum Bot{
        SUPER_DWARAKA
    }
    //Pivot
    public double pivotP;
    public double pivotI;
    public double pivotD;
    public double pivotGearRatio;
    public Motors pivotMotor;
    public double pivotTicksPerRev;
    public double pivotPositionTolerance = 0.1;

    //Viper
    public double viperP;
    public double viperI;
    public double viperD;
    public double viperGearRatio;
    public Motors viperMotor;
    public double viperTicksPerRev;
    public double viperPositionTolerance = 0.1;
    public double viperPulleyDiameter;
    public double viperInchesPerTick;

    public static RobotHardware getInstance() {
        switch(RobotHardware.selectedRobot) {
            case SUPER_DWARAKA:
                return RobotHardwareSuperDwaraka.getInstance();
            default:
                return RobotHardwareSuperDwaraka.getInstance();
        }
    }

    public static void setRobot(RobotHardware.Bot selectedRobot) {
        RobotHardware.selectedRobot = selectedRobot;
        RobotHardware.robot = RobotHardware.getInstance();
    }
}
