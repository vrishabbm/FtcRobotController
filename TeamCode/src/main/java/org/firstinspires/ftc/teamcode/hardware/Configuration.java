package org.firstinspires.ftc.teamcode.hardware;

import org.firstinspires.ftc.teamcode.hardware.configurations.ConfigurationOne;

public class Configuration {
    public static Configuration config = Config.CONFIG_ONE.getConfiguration();

    public enum Config {
        CONFIG_ONE(ConfigurationOne.getInstance());

        private final Configuration configuration;

        private Config(Configuration configuration) {
            this.configuration = configuration;
        }

        private Configuration getConfiguration() {
            return configuration;
        }
    }
    public String viperMotorName;
    public String pivotMotorName;

    public static void setConfiguration(Config selectedConfiguration) {
        Configuration.config = selectedConfiguration.getConfiguration();
    }
}
