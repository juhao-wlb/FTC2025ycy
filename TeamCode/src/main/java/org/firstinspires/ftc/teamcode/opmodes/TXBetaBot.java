package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.StartUntilCommand;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.LiftClaw;

@TeleOp(name = "TXTeleop")
public class TXBetaBot extends CommandOpMode {
    private GamepadEx gamepadEx1;
    private Lift lift;
    private LiftClaw liftClaw;
    @Override
    public void initialize() {
        gamepadEx1 = new GamepadEx(gamepad1);

        lift = new Lift(hardwareMap);
        liftClaw = new LiftClaw(hardwareMap);

//        gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
//            new StartUntilCommand(
//                    () -> lift.getCurrentPosition() > 200,
//                    () -> lift.setGoal(Lift.Goal.BASKET),
//                    liftClaw::upLiftArm
//            )
//        );

        gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new ParallelCommandGroup(
                        new InstantCommand(() -> lift.setGoal(Lift.Goal.BASKET)),
                        new WaitUntilCommand(() -> lift.getCurrentPosition() > 200)
                                .andThen(new InstantCommand(liftClaw::upLiftArm))
                )
        );

        gamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new InstantCommand(liftClaw::switchLiftClaw)
        );

        gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(liftClaw::openClaw),
                        new WaitCommand(100),
                        new InstantCommand(liftClaw::foldLiftArm),
                        new WaitCommand(500),
                        new InstantCommand(() -> lift.setGoal(Lift.Goal.STOW))
                )
        );
    }
}
