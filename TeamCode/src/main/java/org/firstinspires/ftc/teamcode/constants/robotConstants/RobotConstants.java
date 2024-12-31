package org.firstinspires.ftc.teamcode.constants.robotConstants;

import org.firstinspires.ftc.teamcode.constants.CurrentRobot;

public class RobotConstants {
    //Pivot
    public double pivot_kP;
    public double pivot_kI;
    public double pivot_kD;

    //Viper
    public double viper_kP;
    public double viper_kI;
    public double viper_kD;

    public static RobotConstants getInstance(CurrentRobot.Bot selectedRobot) {
        switch(selectedRobot) {
            case SUPER_DWARAKA:
                return SuperDwarakaConstants.getInstance();
            default:
                return SuperDwarakaConstants.getInstance();
        }
    }
}
