package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.subsystems.AlphaClaw;
import org.firstinspires.ftc.teamcode.subsystems.AlphaLift;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;

@TeleOp(name = "ycyAlphaTeleOP")
public class AlphaCar extends LinearOpMode {
//    private Servo clawServo, clawTurnServo, slideServo;
//    private clawTurnServoState turnState = clawTurnServoState.ORIGIN;
    private SampleMecanumDrive drive;
    private AlphaClaw claw;
    private AlphaLift lift;

    @Override
    public void runOpMode() throws InterruptedException {
        double position = 0;
        claw = new AlphaClaw(hardwareMap);
        lift = new AlphaLift(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        ToggleBoolean isRightDPadPressed = new ToggleBoolean(() -> gamepad1.dpad_right);
        ToggleBoolean isLeftDPadPressed = new ToggleBoolean(() -> gamepad1.dpad_left);
        waitForStart();
        while(opModeIsActive()){

            drive.setFieldRelativeDrivePower(new Pose2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            ));

            if(gamepad1.dpad_up) {
                lift.setOpenLoop(1);
            }else if(gamepad1.dpad_down) {
                lift.setOpenLoop(-1);
            }else lift.setOpenLoop(0);
            if(gamepad1.left_bumper) claw.aim(1, position);
            else if(gamepad1.right_bumper) claw.retract();

            boolean rightDPad = isRightDPadPressed.isPressed();

            if(gamepad1.a) {
                drive.resetHeading();
            }
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
            telemetry.addData("fun:",gamepad1.left_stick_x);
            telemetry.addData("turn:",gamepad1.right_stick_x);
//            telemetry.addData("clawServo:",clawServo.getPosition());
            telemetry.addData("heading:",drive.getHeading());
            telemetry.addData("isRightDPadPressed:",rightDPad);
            telemetry.addData("perv Button:",isRightDPadPressed.getLastButton());
            telemetry.update();
        }
    }

    private enum clawTurnServoState {
        ORIGIN(1),
        RIGHT30(0.8),
        RIGHT60(0.6),
        RIGHT90(0.4);

        public final double turnPosition;

        clawTurnServoState(double position) {
            this.turnPosition = position;
        }
    }
}
