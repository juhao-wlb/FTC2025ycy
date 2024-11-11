package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.gobilda.GoBildaPinpointDriver;

@TeleOp(name = "ycyBetaTeleOP")
public class BetaNovaCar extends LinearOpMode {
    private Servo clawServo, turetServo, armServo;
    private DcMotor liftMotor, leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor;
    private TuretServoState turnState = TuretServoState.ORIGIN;
    //private GoBildaPinpointDriver od;
    private double yawOffset,Kp,Ki,Kd;
    private PIDController liftUp = new PIDController(Kp,Ki,Kd), liftBack = new PIDController(Kp,Ki,Kd);



    @Override
    public void runOpMode() throws InterruptedException {
        //clawServo = hardwareMap.get(Servo.class,"clawServo");
        //turetServo = hardwareMap.get(Servo.class,"turetServo");
        //armServo = hardwareMap.get(Servo.class,"armServo");
        liftMotor = hardwareMap.get(DcMotor.class,"liftMotor");
        //leftFrontMotor = hardwareMap.get(DcMotor.class,"leftFrontMotor");
        //leftBackMotor = hardwareMap.get(DcMotor.class,"leftBackMotor");
        //rightFrontMotor = hardwareMap.get(DcMotor.class,"rightFrontMotor");
        //rightBackMotor = hardwareMap.get(DcMotor.class,"rightBackMotor");
        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //od = hardwareMap.get(GoBildaPinpointDriver.class,"od");
        //od.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        //od.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //od.setOffsets(0, 0);



        waitForStart();
        while(opModeIsActive()){
            //od.update();
            double forward = -gamepad1.left_stick_y;
            double fun = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            /*double botHeading = od.getHeading() % 360 - yawOffset;

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
            */
            if(gamepad1.dpad_up) {
                liftMotor.setPower(1);
            }
            else if(gamepad1.dpad_down) {
                liftMotor.setPower(-1);
            }
            else liftMotor.setPower(0);
            /*double upPower = liftUp.calculate(liftMotor.getCurrentPosition(),liftSetPoint);
            double backPower = liftBack.calculate(liftMotor.getCurrentPosition(),liftSetPoint);
            if(gamepad1.dpad_up) {
                liftMotor.setPower(upPower);
            }
            else if(gamepad1.dpad_down) {
                liftMotor.setPower(backPower);
            }
            if(gamepad1.a) {
                yawOffset = od.getHeading() % 360;
            }*/
            /*if(gamepad1.x) {
                armServo.setPosition(0);
            }
            else if(gamepad1.y) {
                armServo.setPosition(1);
            }
            else if(gamepad1.dpad_right) {
                turnState = TuretServoState.ORIGIN;
            }
            else if(gamepad1.dpad_left) {
                turnState = TuretServoState.OPPOSITE;
            }
            turetServo.setPosition(turnState.turnPosition);

            if(gamepad1.right_bumper) {
                clawServo.setPosition(0);
            }
            else if(gamepad1.left_bumper) {
                clawServo.setPosition(1);
            }

            telemetry.addData("forward",gamepad1.left_stick_y);
            telemetry.addData("fun:",gamepad1.left_stick_x);
            telemetry.addData("turn:",gamepad1.right_stick_x);
            //telemetry.addData("heading:",botHeading);
            //telemetry.addData("rawHeading:",od.getHeading());
            telemetry.update();*/
        }
    }

    private enum TuretServoState {
        ORIGIN(0),
        IDLE(0),
        OPPOSITE(1);

        public final double turnPosition;

        TuretServoState(double position) {
            this.turnPosition = position;
        }
    }
}
