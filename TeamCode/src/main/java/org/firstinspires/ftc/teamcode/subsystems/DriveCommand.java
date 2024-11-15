package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lib.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.lib.roadrunner.drive.SampleMecanumDrive;

public class DriveCommand extends CommandBase {
    private final SampleMecanumDrive drive;
    private final Trajectory trajectory;

    public DriveCommand(SampleMecanumDrive mecanumDrive, Trajectory traj) {
        drive = mecanumDrive;
        trajectory = traj;
    }

    @Override
    public void initialize() {
        drive.setPoseEstimate(trajectory.start());
        drive.followTrajectoryAsync(trajectory);
    }

    @Override
    public void execute() {
        drive.update();
    }

    @Override
    public void end(boolean interrupted) {

    }

    public boolean isFinished() {
        return !drive.isBusy();
    }

}
