package org.firstinspires.ftc.teamcode.constants.robotConstants;

import org.firstinspires.ftc.teamcode.constants.CurrentRobot;

public class RobotConstants {
    //Pivot
    public double pivot_kP;
    public double pivot_kI;
    public double pivot_kD;
    public int pivot_motor_encoder_resolution;

    //Viper
    public double viper_kP;
    public double viper_kI;
    public double viper_kD;
    public int viper_motor_encoder_resolution;

    public static RobotConstants getInstance(CurrentRobot.Bot selectedRobot) {
        switch(selectedRobot) {
            case SUPER_DWARAKA:
                return SuperDwarakaConstants.getInstance();
            default:
                return SuperDwarakaConstants.getInstance();
        }
    }
}
