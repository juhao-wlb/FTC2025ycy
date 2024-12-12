package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.subsystems.AlphaClaw;
import org.firstinspires.ftc.teamcode.subsystems.AlphaLift;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;

@Config @TeleOp(name = "ycyAlphaTeleOP")
public class AlphaCar extends LinearOpMode {
    public static boolean isHeadless = true;
    public static double maxPower = 1;
//    private Servo clawServo, clawTurnServo, slideServo;
//    private clawTurnServoState turnState = clawTurnServoState.ORIGIN;
    private SampleMecanumDrive drive;
    private AlphaClaw claw;
    private AlphaLift lift;
    private GamepadEx gamepadEx1;
    static{
        headless = new Gamepad.RumbleEffect.Builder().addStep(0.5, 0.1, 250)
                .addStep(0, 0.5, 250)
                .build();
        headnormal = new Gamepad.RumbleEffect.Builder().addStep(1, 1, 250)
                .addStep(0, 0, 250)
                .addStep(1, 1, 250)
                .build();
    }
    private static final Gamepad.RumbleEffect headless;
    private static final Gamepad.RumbleEffect headnormal;

    @Override
    public void runOpMode() throws InterruptedException {
        double position = 0;
        claw = new AlphaClaw(hardwareMap);
        lift = new AlphaLift(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        gamepadEx1 = new GamepadEx(gamepad1);
        ToggleBoolean isRightDPadPressed = new ToggleBoolean(() -> gamepad1.dpad_right);
        ToggleBoolean isLeftDPadPressed = new ToggleBoolean(() -> gamepad1.dpad_left);
        gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenHeld(
                new SequentialCommandGroup(
                        new InstantCommand(drive::resetHeading),
                        new WaitCommand(300),
                        new InstantCommand(() -> {
                            isHeadless=!isHeadless;
                            if(isHeadless){
                                gamepad1.runRumbleEffect(headless);
                            }else{
                                gamepad1.runRumbleEffect(headnormal);
                            }
                        }))
        );
        gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new InstantCommand(() -> {
                    if(maxPower==0.3){
                        maxPower = 1;
                    }else{
                        maxPower = 0.3;
                    }
                })
        );
        waitForStart();
        while(opModeIsActive()){
            if(isHeadless){
                drive.setFieldRelativeDrivePower(new Pose2d(
                        -gamepad1.left_stick_y * maxPower,
                        -gamepad1.left_stick_x * maxPower,
                        -gamepad1.right_stick_x * maxPower
                ));
            }else{
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y * maxPower,
                                -gamepad1.left_stick_x * maxPower,
                                -gamepad1.right_stick_x *maxPower
                        )
                );
            }

            if(gamepad1.dpad_up) {
                lift.setOpenLoop(1);
//                lift.setSetPoint(0.5);
            }else if(gamepad1.dpad_down) {
                lift.setOpenLoop(-1);
//                lift.setSetPoint(0);
            }else lift.setOpenLoop(0);
            if(gamepad1.left_bumper) claw.aim(1, position);
            else if(gamepad1.right_bumper) claw.retract();

            boolean rightDPad = isRightDPadPressed.isPressed();

            if(rightDPad) {
                telemetry.addLine("RD Pressed");
                position += 1.0/12;
                claw.setYaw(position);
            }
            if(isLeftDPadPressed.isPressed()) {
                telemetry.addLine("LD Pressed");
                position -= 1.0/12;
                claw.setYaw(position);
            }

            if(gamepad1.x) {
                claw.release();
            }
            else if(gamepad1.y) {
                claw.grab();
            }
            telemetry.addData("forward",gamepad1.left_stick_y);
            telemetry.addData("fun",gamepad1.left_stick_x);
            telemetry.addData("turn",gamepad1.right_stick_x);
//            telemetry.addData("clawServo",clawServo.getPosition());
            telemetry.addData("heading",drive.getHeading());
            telemetry.addData("isRightDPadPressed",rightDPad);
            telemetry.addData("perv Button",isRightDPadPressed.getLastButton());
            telemetry.update();
            CommandScheduler.getInstance().run();
        }
        CommandScheduler.getInstance().reset();
    }
}
