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
import org.firstinspires.ftc.teamcode.hardware.configurations.Configuration;
import org.firstinspires.ftc.teamcode.utils.MathUtil;
import org.firstinspires.ftc.teamcode.utils.PIDFController;

public class Viper implements Subsystem{
    private DcMotorEx viperMotor;
    private double currentPosition;
    private double setpoint;
    private PIDFController pidfController;


    @Override
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        viperMotor = hardwareMap.get(DcMotorEx.class, Configuration.viperMotorName);

        viperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viperMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        pidfController = new PIDFController(RobotHardware.robot.viperP, RobotHardware.robot.viperI, RobotHardware.robot.viperD, 0);
        pidfController.setTolerance(RobotHardware.robot.viperPositionTolerance);
    }

    @Override
    public void sendTelemetry(Telemetry telemetry) {
        telemetry.addData("Viper Position (in)", currentPosition);
    }

    @Override
    public void periodic(Telemetry telemetry) {
        currentPosition = viperMotor.getCurrentPosition()/RobotHardware.robot.viperInchesPerTick;
    }

    @Override
    public void stop(Telemetry telemetry) {
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
        class MoveViper implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                currentPosition = viperMotor.getCurrentPosition()/RobotHardware.robot.viperInchesPerTick;

                viperMotor.setPower(MathUtil.clamp(pidfController.calculate(currentPosition, setpoint), -1, 1));

                if(Math.abs(currentPosition-setpoint) > RobotHardware.robot.viperPositionTolerance) {
                    return true;
                } else {
                    viperMotor.setPower(0);
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
