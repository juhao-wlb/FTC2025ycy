package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lib.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.lib.roadrunner.drive.SampleMecanumDrive;

public class DriveCommand extends CommandBase {
    private final SampleMecanumDrive drive;

    public DriveCommand(SampleMecanumDrive mecanumDrive) {
        drive = mecanumDrive;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    public boolean isFinished() {
        return false;
    }

}
