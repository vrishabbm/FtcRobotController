package org.firstinspires.ftc.teamcode.constants;

import org.firstinspires.ftc.teamcode.constants.robotConstants.RobotConstants;

public class CurrentRobot {
    private static Bot selectedBot = Bot.SUPER_DWARAKA;
    public enum Bot{
        SUPER_DWARAKA
    }
    static RobotConstants bot = RobotConstants.getInstance(selectedBot);
    public static double pivot_kP = bot.pivot_kP;
    public static double pivot_kI = bot.pivot_kI;
    public static double pivot_kD = bot.pivot_kD;
    public static int pivot_motor_encoder_resolution = bot.pivot_motor_encoder_resolution;

    public static double viper_kP = bot.viper_kP;
    public static double viper_kI = bot.viper_kI;
    public static double viper_kD = bot.viper_kD;
    public static int viper_motor_encoder_resolution = bot.viper_motor_encoder_resolution;
}
