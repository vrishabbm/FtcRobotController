package org.firstinspires.ftc.teamcode.hardware.configurations;

import org.firstinspires.ftc.teamcode.hardware.Configuration;

public class ConfigurationOne extends Configuration {
    private static Configuration instance;

    public static Configuration getInstance() {
        if(instance == null) {
            instance = new ConfigurationOne();
        }

        return instance;
    }

    public ConfigurationOne() {
        viperMotorName = "viper";
        pivotMotorName = "pivot";
    }
}
