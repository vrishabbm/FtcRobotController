package org.firstinspires.ftc.teamcode.hardware;

public enum Motors {
    GoBilda_5203_312RPM(537.7,312);

    private final double ENCODER_RES, MAX_RPM, MAX_VEL_PPS;

    private Motors(double encoderResolution, double maxRPM) {
        this.ENCODER_RES = encoderResolution;
        this.MAX_RPM = maxRPM;
        this.MAX_VEL_PPS = ENCODER_RES * MAX_RPM / 60;
    }

    public double getEncoderResolution() {
        return ENCODER_RES;
    }

    public double getMaxRPM() {
        return MAX_RPM;
    }

    public double getMaxVelPPS() {
        return  MAX_VEL_PPS;
    }
}
