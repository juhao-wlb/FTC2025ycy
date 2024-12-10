package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopDriveCommand extends CommandBase {
    private final MecanumDrive drive;
    private final DoubleSupplier forward;
    private final DoubleSupplier rotate;
    private final DoubleSupplier fun;
    private final BooleanSupplier shouldReset;

    public TeleopDriveCommand(MecanumDrive drive,
                              DoubleSupplier forward,
                              DoubleSupplier fun,
                              DoubleSupplier rotate,
                              BooleanSupplier shouldReset) {
        this.drive = drive;
        this.forward = forward;
        this.rotate = rotate;
        this.fun = fun;
        this.shouldReset = shouldReset;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        if (shouldReset.getAsBoolean()) {
            drive.reset();
        }
        drive.moveRobot(forward.getAsDouble(), fun.getAsDouble(), rotate.getAsDouble());
    }

}
