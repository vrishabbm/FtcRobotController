package org.firstinspires.ftc.teamcode.constants.robotConstants;

public class SuperDwarakaConstants extends RobotConstants{
    private static SuperDwarakaConstants instance;
    public SuperDwarakaConstants() {
        pivot_kP = 0.1;
        pivot_kI = 0.0;
        pivot_kD = 0.0004;
        pivot_motor_encoder_resolution = 300;

        viper_kP = 0.1;
        viper_kI = 0.0;
        viper_kD = 0.0004;
        viper_motor_encoder_resolution = 300;
    }

    public static SuperDwarakaConstants getInstance() {
        if (instance == null) {
            instance = new SuperDwarakaConstants();
        }

        return instance;
    }
}
