package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.CurrentRobot;
import org.firstinspires.ftc.teamcode.constants.subsystemConstants.PivotConstants;
import org.firstinspires.ftc.teamcode.utils.MathUtil;
import org.firstinspires.ftc.teamcode.utils.PIDFController;

public class Viper implements Subsystem{
    private DcMotorEx viperMotor;
    private double currentPosition;
    PIDFController pidfController;

    //TODO: Transfer constants to a constants class
    private final double PULLEY_DIAMETER_INCHES = 4.4; //estimate
    private final double VIPER_MOTOR_TICKS_PER_REV = 400; //estimate
    private final double VIPER_POSITION_TOLERANCE = 0.1;


    @Override
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        viperMotor = hardwareMap.get(DcMotorEx.class, PivotConstants.pivotMotorName);

        viperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viperMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        pidfController = new PIDFController(CurrentRobot.viper_kP, CurrentRobot.viper_kI, CurrentRobot.viper_kD, 0);
        pidfController.setTolerance(VIPER_POSITION_TOLERANCE);
    }

    @Override
    public void sendTelemetry(Telemetry telemetry) {
        telemetry.addData("Viper Position", currentPosition);
    }

    @Override
    public void periodic(Telemetry telemetry) {
        currentPosition = PULLEY_DIAMETER_INCHES * viperMotor.getCurrentPosition()/VIPER_MOTOR_TICKS_PER_REV;
    }

    @Override
    public void stop(Telemetry telemetry) {
        viperMotor.setPower(0);
    }

    public Action extendSlide(double setpoint) {
        class extendSlide implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                viperMotor.setPower(MathUtil.clamp(pidfController.calculate(currentPosition, setpoint), -1, 1));
                return Math.abs(currentPosition-setpoint) > VIPER_POSITION_TOLERANCE;
            }
        }

        return new extendSlide();
    }

    public Action stopViper() {
        return new InstantAction(() -> viperMotor.setPower(0));
    }
}
