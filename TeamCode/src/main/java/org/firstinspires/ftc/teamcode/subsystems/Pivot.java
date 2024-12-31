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

public class Pivot implements Subsystem{
    private DcMotorEx pivotMotor;
    private double currentPosition;
    PIDFController pidfController;

    public Pivot() {
    }

    @Override
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        pivotMotor = hardwareMap.get(DcMotorEx.class, PivotConstants.pivotMotorName);

        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        pidfController = new PIDFController(CurrentRobot.pivot_kP, CurrentRobot.pivot_kI, CurrentRobot.pivot_kD, 0);
        pidfController.setTolerance(PivotConstants.PIVOT_POSITION_TOLERANCE);
    }

    @Override
    public void sendTelemetry(Telemetry telemetry) {
        telemetry.addData("Pivot Position", currentPosition);
    }

    @Override
    public void periodic(Telemetry telemetry) {
        currentPosition = 360 * pivotMotor.getCurrentPosition()/PivotConstants.PIVOT_MOTOR_TICKS_PER_REV;
    }

    @Override
    public void stop(Telemetry telemetry) {
        pivotMotor.setPower(0);
    }

    public Action pivotArm(double setpoint) {
        class PivotArm implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                pivotMotor.setPower(MathUtil.clamp(pidfController.calculate(currentPosition, setpoint), -1, 1));
                return Math.abs(currentPosition-setpoint) > PivotConstants.PIVOT_POSITION_TOLERANCE;
            }
        }

        return new PivotArm();
    }

    public Action stopPivot() {
        return new InstantAction(() -> pivotMotor.setPower(0));
    }
}
