package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Configuration;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Sample Auton")
public class SampleAuton extends LinearOpMode {
    private List<Action> runningActions = new ArrayList<>();
    MecanumDrive drive;
    Action trajectory1 = drive.actionBuilder(new Pose2d(0,0,0))
            .lineToX(37)
            .build();

    Action trajectory2 = drive.actionBuilder(new Pose2d(37,0,0))
            .lineToX(-27)
            .lineToY(-10)
            .build();

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware.setRobot(RobotHardware.Bot.SUPER_DWARAKA);
        Configuration.setConfiguration(Configuration.Config.CONFIG_ONE);
        RobotContainer.initialize(hardwareMap, telemetry);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        waitForStart();

        Actions.runBlocking(new SequentialAction(
                    new ParallelAction(
                            trajectory1,
                            RobotContainer.reachHighBasket()
                    ),
                    trajectory2,
                    RobotContainer.zeroPivotAndViper()
                )
        );

        RobotContainer.stop();
    }
}
