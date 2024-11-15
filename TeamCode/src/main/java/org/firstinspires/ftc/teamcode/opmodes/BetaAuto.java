package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.onbotjava.handlers.objbuild.WaitForBuild;
import org.firstinspires.ftc.teamcode.lib.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.TrajectoryManager;

import java.util.Optional;

@Autonomous(name = "BetaAuto")
public class BetaAuto extends LinearOpMode {
    Pose2d startPoint = new Pose2d(0,0);
    TrajectorySequence trajs1 = TrajectoryManager.trajectorySequenceBuilder(startPoint)
            .lineToConstantHeading(new Vector2d(5,0))
            .turn(Math.toRadians(180))
            .lineToConstantHeading(new Vector2d(0,0))
            .build();




    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        CommandScheduler.getInstance().schedule(
                new DriveCommand(drive, Optional.ofNullable(null), Optional.ofNullable(trajs1))

        );

        waitForStart();
        while(opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();
        }



    }
}
