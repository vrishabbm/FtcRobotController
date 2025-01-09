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
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.Configuration;
import org.firstinspires.ftc.teamcode.utils.MathUtil;
import org.firstinspires.ftc.teamcode.utils.PIDFController;

public class Viper implements Subsystem{
    private DcMotorEx viperMotor;
    private double currentPosition;
    private double setpoint;
    private PIDFController pidfController;
    private static Viper instance;

    public static Viper getInstance() {
        if(instance == null) {
            instance = new Viper();
        }

        return instance;
    }

    public Viper() {
        pidfController = new PIDFController(RobotHardware.robot.viperP, RobotHardware.robot.viperI, RobotHardware.robot.viperD, 0);
        pidfController.setTolerance(RobotHardware.robot.viperPositionTolerance);
    }
    @Override
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        viperMotor = hardwareMap.get(DcMotorEx.class, Configuration.config.viperMotorName);

        viperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viperMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void sendTelemetry(Telemetry telemetry) {
        telemetry.addData("Viper Position (in)", currentPosition);
    }

    @Override
    public void periodic() {
        currentPosition = viperMotor.getCurrentPosition()/RobotHardware.robot.viperInchesPerTick;
        viperMotor.setPower(MathUtil.clamp(pidfController.calculate(currentPosition, setpoint), -1, 1));
    }

    @Override
    public void stop() {
        viperMotor.setPower(0);
    }

    //Getters and Setters
    public double getCurrentPosition() {
        return currentPosition;
    }

    public boolean viperAtSetpoint() {
        return Math.abs(currentPosition - setpoint) < RobotHardware.robot.viperPositionTolerance;
    }

    public double getSetpoint(double setpoint) {
        return setpoint;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }


    //Actions
    public Action moveViper(double setpoint) {
        this.setSetpoint(setpoint);
        class MoveViper implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                Viper.getInstance().periodic();
                if(Math.abs(currentPosition-setpoint) > RobotHardware.robot.viperPositionTolerance) {
                    return true;
                } else {
                    Viper.getInstance().stop();
                    return false;
                }
            }
        }

        return new MoveViper();
    }

    public Action stopViper() {
        return new InstantAction(() -> viperMotor.setPower(0));
    }
}
