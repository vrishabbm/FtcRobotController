package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Configuration;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Sample Teleop")
public class SampleTeleop extends OpMode {
    private List<Action> runningActions = new ArrayList<>();
    MecanumDrive drive;

    @Override
    public void init() {
        RobotHardware.setRobot(RobotHardware.Bot.SUPER_DWARAKA);
        Configuration.setConfiguration(Configuration.Config.CONFIG_ONE);
        RobotContainer.initialize(hardwareMap, telemetry);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
    }

    @Override
    public void loop() {
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-gamepad1.left_stick_y, gamepad1.left_stick_x), gamepad1.right_stick_x));

        RobotContainer.periodic();

        if(gamepad2.y) {
            runningActions.add(RobotContainer.reachHighBasket());
        }

        if(gamepad2.a) {
            runningActions.add(RobotContainer.zeroPivotAndViper());
        }

        TelemetryPacket packet = new TelemetryPacket();

        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;

        RobotContainer.sendTelemetry(telemetry);
    }

    @Override
    public void stop() {
        RobotContainer.stop();
    }
}
