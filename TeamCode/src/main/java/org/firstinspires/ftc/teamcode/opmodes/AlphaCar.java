package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;

@TeleOp(name = "ycyAlphaTeleOP")
public class AlphaCar extends LinearOpMode {
    private Servo clawServo, clawTurnServo;
    private DcMotor slideMotor, leftLiftMotor, rightLiftMotor;
    private clawTurnServoState turnState = clawTurnServoState.ORIGIN;
    private SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        clawServo = hardwareMap.get(Servo.class,"clawServo");
        clawTurnServo = hardwareMap.get(Servo.class,"clawTurnServo");
        slideMotor = hardwareMap.get(DcMotor.class,"slideMotor");
        leftLiftMotor = hardwareMap.get(DcMotor.class,"leftLiftMotor");
        rightLiftMotor = hardwareMap.get(DcMotor.class,"rightLiftMotor");
        leftLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
                leftLiftMotor.setPower(1);
                rightLiftMotor.setPower(1);
            }
            else if(gamepad1.dpad_down) {
                leftLiftMotor.setPower(-1);
                rightLiftMotor.setPower(-1);
            }
            else {
                leftLiftMotor.setPower(0);
                rightLiftMotor.setPower(0);
            }
            if(gamepad1.left_bumper) slideMotor.setPower(1);
            else if(gamepad1.right_bumper) slideMotor.setPower(-1);
            else slideMotor.setPower(0);

            boolean rightDPad = isRightDPadPressed.isPressed();

            if(gamepad1.a) {
                drive.resetHeading();
            }
            if(rightDPad) {
                telemetry.addLine("RD Pressed");
                switch(turnState) {
                    case ORIGIN:
                        turnState = clawTurnServoState.RIGHT30;
                        break;
                    case RIGHT90:
                        turnState = clawTurnServoState.RIGHT90;
                        break;
                    case RIGHT60:
                        turnState = clawTurnServoState.RIGHT90;
                        break;
                    case RIGHT30:
                        turnState = clawTurnServoState.RIGHT60;
                        break;

                }
            }
            if(isLeftDPadPressed.isPressed()) {
                telemetry.addLine("LD Pressed");
                switch(turnState) {
                    case ORIGIN:
                        turnState = clawTurnServoState.ORIGIN;
                        break;
                    case RIGHT90:
                        turnState = clawTurnServoState.RIGHT60;
                        break;
                    case RIGHT60:
                        turnState = clawTurnServoState.RIGHT30;
                        break;
                    case RIGHT30:
                        turnState = clawTurnServoState.ORIGIN;
                        break;
                }
            }

            clawTurnServo.setPosition(turnState.turnPosition);
            if(gamepad1.x) {
                telemetry.addLine("X Pressed");
                clawServo.setPosition(0);
            }
            else if(gamepad1.y) {
                telemetry.addLine("Y Pressed");
                clawServo.setPosition(1);
            }
            telemetry.addData("forward",gamepad1.left_stick_y);
            telemetry.addData("fun:",gamepad1.left_stick_x);
            telemetry.addData("turn:",gamepad1.right_stick_x);
            telemetry.addData("clawServo:",clawServo.getPosition());
            telemetry.addData("heading:",drive.getHeading());
            telemetry.addData("slideMotor:",clawServo.getPosition());
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
