package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.MathUtil;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.lib.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.opmodes.autos.AutoCommand;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.LiftClaw;
import org.firstinspires.ftc.teamcode.subsystems.SlideSuperStucture;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.MathUtils;

@Autonomous(name="Autowlb")
public class Autowlb extends LinearOpMode {
    @Override
    public void runOpMode() {
        Lift lift=new Lift(hardwareMap,telemetry);
        LiftClaw liftClaw=new LiftClaw(hardwareMap);
        SlideSuperStucture slide = new SlideSuperStucture(hardwareMap,telemetry);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence startTrajectory = drive.trajectorySequenceBuilder(new Pose2d())
                .lineTo(new Vector2d(33,16))
                .build();
        TrajectorySequence grabTrajectory = drive.trajectorySequenceBuilder(startTrajectory.end())
                .lineToLinearHeading(new Pose2d(1,16, Math.toRadians(135)))
                .build();
        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
//                AutoCommand.initialize(liftClaw, slide),
//                new AutoDriveCommand(drive,startTrajectory),
                slide.grabCommand()
//                new AutoDriveCommand(drive,grabTrajectory),
//                AutoCommand.autoFinish(liftClaw,lift,slide)
        ));
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();
        }
        CommandScheduler.getInstance().reset();
    }
}
