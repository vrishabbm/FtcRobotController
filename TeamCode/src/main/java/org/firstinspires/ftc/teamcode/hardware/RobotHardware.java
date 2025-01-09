package org.firstinspires.ftc.teamcode.hardware;

import org.firstinspires.ftc.teamcode.hardware.robots.RobotHardwareSuperDwaraka;

public class RobotHardware {
    public static RobotHardware robot = Bot.SUPER_DWARAKA.getBot();

    public enum Bot{
        SUPER_DWARAKA(RobotHardwareSuperDwaraka.getInstance());

        private final RobotHardware bot;

        private Bot(RobotHardware bot) {
            this.bot = bot;
        }

        public RobotHardware getBot() {
            return this.bot;
        }
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

    public static void setRobot(RobotHardware.Bot selectedRobot) {
        RobotHardware.robot = selectedRobot.getBot();
    }
}
