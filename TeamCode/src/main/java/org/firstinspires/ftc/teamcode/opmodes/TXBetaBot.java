package org.firstinspires.ftc.teamcode.opmodes;

import android.transition.Slide;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.TeleopDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.LiftClaw;
import org.firstinspires.ftc.teamcode.subsystems.SlideSuperStucture;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.FunctionalButton;

@TeleOp(name = "TXTeleop")
@Config
public class TXBetaBot extends CommandOpMode {
    private GamepadEx gamepadEx1;
    private Lift lift;
    private LiftClaw liftClaw;
    private SlideSuperStucture slide;
    private MecanumDrive drive;

    public static boolean isRaw = false;

    @Override
    public void initialize() {
        gamepadEx1 = new GamepadEx(gamepad1);

        lift = new Lift(hardwareMap);
        liftClaw = new LiftClaw(hardwareMap);
        slide = new SlideSuperStucture(hardwareMap, telemetry);
        drive = new MecanumDrive(hardwareMap);

        drive.setDefaultCommand(new TeleopDriveCommand(
                drive, () -> -gamepadEx1.getLeftY(),
                () -> -gamepadEx1.getLeftX(), () -> gamepadEx1.getRightX(),
                () -> gamepadEx1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
        ));

        gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new ParallelCommandGroup(
                        new InstantCommand(() -> lift.setGoal(Lift.Goal.BASKET)),
                        new WaitUntilCommand(() -> lift.getCurrentPosition() > 600)
                                .andThen(new InstantCommand(liftClaw::upLiftArm))
                )
        );

//        gamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
//                new InstantCommand(liftClaw::switchLiftClaw)
//        );

        gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(liftClaw::openClaw),
                        new WaitCommand(100),
                        new InstantCommand(liftClaw::foldLiftArm),
                        new WaitCommand(500),
                        new InstantCommand(() -> lift.setGoal(Lift.Goal.STOW))
                )
        );

        gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                slide.aimCommand().alongWith(new InstantCommand(() -> liftClaw.openClaw())), false
        );

        //gamepadEx1.getGamepadButton(GamepadKeys.Button.A).and(new Trigger(() -> slide.getGoal() == SlideSuperStucture.Goal.AIM).whenActive(slide.grabCommand(), false));

        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.A)
                        && slide.getGoal() == SlideSuperStucture.Goal.AIM)
                .whenPressed(slide.grabCommand(), false);

        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.DPAD_RIGHT)
                        && slide.getGoal() == SlideSuperStucture.Goal.AIM)
                .whenPressed(slide.handoffCommand(isRaw)
                        .andThen(new WaitCommand(500))
                        .andThen(new InstantCommand(() -> liftClaw.closeClaw()))
                        .andThen(new WaitCommand(300))
                        .andThen(new InstantCommand(() -> slide.openIntakeClaw())), false);


//        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
//                .and(new Trigger(() -> slide.getGoal() == SlideSuperStucture.Goal.AIM))
//                .whenActive(
//                slide.handoffCommand().andThen(new WaitCommand(1000))
//                        .andThen(new InstantCommand(() -> liftClaw.closeClaw())), false
//        );

        new FunctionalButton(
                () -> gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5
                    && slide.getGoal() != SlideSuperStucture.Goal.HANDOFF)
                .whenHeld(
                    new FunctionalCommand(
                            ()->{},
                            () -> slide.forwardSlideExtension(),
                            (bool) ->{},
                            () -> false)
        );

        new FunctionalButton(
                () -> gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5
                    && slide.getGoal() != SlideSuperStucture.Goal.HANDOFF)
                .whenHeld(
                        new FunctionalCommand(
                                ()->{},
                                () -> slide.backwardSlideExtension(),
                                (bool) ->{},
                                () -> false)
        );

        gamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new InstantCommand(() -> slide.leftTurnServo())
        );

        gamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new InstantCommand(() -> slide.rightTurnServo())
        );

    }


}
