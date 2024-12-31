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

public class Pivot implements Subsystem{
    private DcMotorEx pivotMotor;
    private double currentPosition;
    private double setpoint;
    private PIDFController pidfController;

    public Pivot() {
        pidfController = new PIDFController(RobotHardware.robot.pivotP, RobotHardware.robot.pivotI, RobotHardware.robot.pivotD, 0);
        pidfController.setTolerance(RobotHardware.robot.pivotPositionTolerance);
    }

    @Override
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        pivotMotor = hardwareMap.get(DcMotorEx.class, Configuration.pivotMotorName);
        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void sendTelemetry(Telemetry telemetry) {
        telemetry.addData("Pivot Position (degrees)", currentPosition);
    }

    @Override
    public void periodic(Telemetry telemetry) {
        currentPosition = 360 * pivotMotor.getCurrentPosition()/RobotHardware.robot.pivotTicksPerRev;
    }

    @Override
    public void stop(Telemetry telemetry) {
        pivotMotor.setPower(0);
    }

    //Getters and Setters
    public double getCurrentPosition() {
        return currentPosition;
    }

    public double getSetpoint(double setpoint) {
        return setpoint;
    }

    public boolean pivotAtSetpoint() {
        return Math.abs(currentPosition - setpoint) < RobotHardware.robot.pivotPositionTolerance;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    //Actions
    public Action pivotArm(double setpoint) {
        this.setSetpoint(setpoint);
        class PivotArm implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                currentPosition = 360 * pivotMotor.getCurrentPosition()/RobotHardware.robot.pivotTicksPerRev;

                pivotMotor.setPower(MathUtil.clamp(pidfController.calculate(currentPosition, setpoint), -1, 1));

                if(Math.abs(currentPosition-setpoint) > RobotHardware.robot.pivotPositionTolerance) {
                    return true;
                } else {
                    pivotMotor.setPower(0);
                    return false;
                }
            }
        }

        return new PivotArm();
    }

    public Action stopPivot() {
        return new InstantAction(() -> pivotMotor.setPower(0));
    }
}
