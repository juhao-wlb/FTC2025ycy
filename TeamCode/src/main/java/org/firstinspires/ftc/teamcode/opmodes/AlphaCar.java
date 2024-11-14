package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.lib.gobilda.GoBildaPinpointDriver;

@TeleOp(name = "ycyAlphaTeleOP")
public class AlphaCar extends LinearOpMode {
    private Servo clawServo, clawTurnServo;
    private DcMotor leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor, slideMotor, leftLiftMotor, rightLiftMotor;
    private clawTurnServoState turnState = clawTurnServoState.ORIGIN;
    private GoBildaPinpointDriver od;
    private double yawOffset;

    @Override
    public void runOpMode() throws InterruptedException {
        clawServo = hardwareMap.get(Servo.class,"clawServo");
        clawTurnServo = hardwareMap.get(Servo.class,"clawTurnServo");
        slideMotor = hardwareMap.get(DcMotor.class,"slideMotor");
        leftLiftMotor = hardwareMap.get(DcMotor.class,"leftLiftMotor");
        rightLiftMotor = hardwareMap.get(DcMotor.class,"rightLiftMotor");
        leftFrontMotor = hardwareMap.get(DcMotor.class,"leftFrontMotor");
        leftBackMotor = hardwareMap.get(DcMotor.class,"leftBackMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class,"rightFrontMotor");
        rightBackMotor = hardwareMap.get(DcMotor.class,"rightBackMotor");
        leftLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        od = hardwareMap.get(GoBildaPinpointDriver.class,"od");
        od.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        od.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        od.setOffsets(155, -25);

        ToggleBoolean isRightDPadPressed = new ToggleBoolean(() -> gamepad1.dpad_right);
        ToggleBoolean isLeftDPadPressed = new ToggleBoolean(() -> gamepad1.dpad_left);

        waitForStart();
        while(opModeIsActive()){
            od.update();
            double forward = -gamepad1.left_stick_y;
            double fun = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            double botHeading = od.getHeading() % 360 - yawOffset;

            // Rotate the movement direction counter to the bot's rotation
            double rotX = fun * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
            double rotY = fun * Math.sin(-botHeading) + forward * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
            double leftFrontPower = (rotY + rotX + turn) / denominator;
            double leftBackPower = (rotY - rotX + turn) / denominator;
            double rightFrontPower = (rotY - rotX - turn) / denominator;
            double rightBackPower = (rotY + rotX - turn) / denominator;
            leftFrontMotor.setPower(leftFrontPower);
            leftBackMotor.setPower(leftBackPower);
            rightFrontMotor.setPower(rightFrontPower);
            rightBackMotor.setPower(rightBackPower);
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
                yawOffset = od.getHeading() % 360;
            }
            else if(rightDPad) {
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
            else if(isLeftDPadPressed.isPressed()) {
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
                clawServo.setPosition(0);
            }
            else if(gamepad1.y) {
                clawServo.setPosition(1);
            }
            telemetry.addData("forward",gamepad1.left_stick_y);
            telemetry.addData("fun:",gamepad1.left_stick_x);
            telemetry.addData("turn:",gamepad1.right_stick_x);
            telemetry.addData("clawServo:",clawServo.getPosition());
            telemetry.addData("heading:",botHeading);
            telemetry.addData("rawHeading:",od.getHeading());
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
