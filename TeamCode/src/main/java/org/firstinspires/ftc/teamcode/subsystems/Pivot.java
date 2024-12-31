package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.subsystemConstants.PivotConstants;

public class Pivot implements Subsystem{
    DcMotorEx pivotMotor;
    double currentPosition;
    double setpoint;


    @Override
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        pivotMotor = hardwareMap.get(DcMotorEx.class, PivotConstants.pivotMotorName);

        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void sendTelemetry(Telemetry telemetry) {

    }

    @Override
    public void periodic(Telemetry telemetry) {

    }

    @Override
    public void stop(Telemetry telemetry) {

    }
}
