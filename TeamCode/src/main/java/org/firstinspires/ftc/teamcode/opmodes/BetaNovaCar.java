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
  private Servo clawServo,
      wristServo,
      wristTurnServo,
      intakeClawServo,
      liftArmServo,
      slideArmServo,
      slideLeftServo,
      slideRightServo;
  private DcMotor liftMotor, leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor;
  private GoBildaPinpointDriver od;
  private double yawOffset, Kp, Ki, Kd, wristTurnPosition = 0.35, slideServoPosition = 1;
  private PIDController liftUp = new PIDController(Kp, Ki, Kd),
      liftBack = new PIDController(Kp, Ki, Kd);

  @Override
  public void runOpMode() throws InterruptedException {
    clawServo = hardwareMap.get(Servo.class, "clawServo");
    wristServo = hardwareMap.get(Servo.class, "wristServo");
    wristTurnServo = hardwareMap.get(Servo.class, "wristTurnServo");
    intakeClawServo = hardwareMap.get(Servo.class, "intakeClawServo");
    liftArmServo = hardwareMap.get(Servo.class, "liftArmServo");
    slideArmServo = hardwareMap.get(Servo.class, "slideArmServo");
    slideLeftServo = hardwareMap.get(Servo.class, "slideLeftServo");
    slideRightServo = hardwareMap.get(Servo.class, "slideRightServo");
    liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
    leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
    leftBackMotor = hardwareMap.get(DcMotor.class, "leftBackMotor");
    rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
    rightBackMotor = hardwareMap.get(DcMotor.class, "rightBackMotor");
    liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    slideLeftServo.setDirection(Servo.Direction.REVERSE);
    od = hardwareMap.get(GoBildaPinpointDriver.class, "od");
    od.setEncoderDirections(
        GoBildaPinpointDriver.EncoderDirection.FORWARD,
        GoBildaPinpointDriver.EncoderDirection.FORWARD);
    od.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    od.setOffsets(0, 0);

    ToggleBoolean isDpadLeft1 = new ToggleBoolean(() -> gamepad1.dpad_left);
    ToggleBoolean isDpadRight1 = new ToggleBoolean(() -> gamepad1.dpad_right);
    ToggleBoolean isDpadLeft2 = new ToggleBoolean(() -> gamepad2.dpad_left);
    ToggleBoolean isDpadRight2 = new ToggleBoolean(() -> gamepad2.dpad_right);
    ToggleBoolean isDpadUp2 = new ToggleBoolean(() -> gamepad2.dpad_up);
    ToggleBoolean isDpadDown2 = new ToggleBoolean(() -> gamepad2.dpad_down);

    waitForStart();
    while (opModeIsActive()) {
      od.update();
      double forward = -gamepad1.left_stick_y;
      double fun = gamepad1.left_stick_x;
      double turn = gamepad1.right_stick_x;

      double botHeading = od.getHeading() - yawOffset;

      // Rotate the movement direction counter to the bot's rotation
      double rotX = fun * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
      double rotY = fun * Math.sin(-botHeading) + forward * Math.cos(-botHeading);

      rotX = rotX * 1.1; // Counteract imperfect strafing

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

      if (gamepad1.dpad_up) {
        liftMotor.setPower(1);
      } else if (gamepad1.dpad_down) {
        liftMotor.setPower(-1);
      } else liftMotor.setPower(0);

      // double upPower = liftUp.calculate(liftMotor.getCurrentPosition(),liftSetPoint);
      // double backPower = liftBack.calculate(liftMotor.getCurrentPosition(),liftSetPoint);

      if (isDpadLeft1.isPressed()) {
        if (slideArmServo.getPosition() >= 0.7) {
          slideArmServo.setPosition(1);
        } else if (slideArmServo.getPosition() <= 0.7) {
          slideArmServo.setPosition(0.9);
        }
      } else if (isDpadRight1.isPressed()) {
        if (slideArmServo.getPosition() <= 0.95) {
          slideArmServo.setPosition(0.5);
        } else if (slideArmServo.getPosition() >= 0.95) {
          slideArmServo.setPosition(0.9);
        }
      }
      if (gamepad1.right_bumper) {
        yawOffset = od.getHeading();
      }
      if (gamepad1.x) {
        liftArmServo.setPosition(1);
      } else if (gamepad1.y) {
        liftArmServo.setPosition(0.25);
      }

      if (gamepad1.a) {
        clawServo.setPosition(0); // Close
      } else if (gamepad1.b) {
        clawServo.setPosition(0.5); // Open
      }

      if (gamepad2.a) {
        wristServo.setPosition(0.05); //
      } else if (gamepad2.b) {
        wristServo.setPosition(0.75);
      }

      if (isDpadLeft2.isPressed() && wristTurnPosition - 0.2 >= 0.15) {
        wristTurnPosition -= 0.2;
      } else if (isDpadRight2.isPressed() && wristTurnPosition + 0.2 <= 0.55) {
        wristTurnPosition += 0.2;
      }
      wristTurnServo.setPosition(wristTurnPosition);

      if (gamepad2.x) {
        intakeClawServo.setPosition(0.3); // Up
      } else if (gamepad2.y) {
        intakeClawServo.setPosition(0.7); // Down
      }

      if (isDpadUp2.isPressed() && slideServoPosition + 0.05 <= 1) {
        slideServoPosition += 0.05;
      } else if (isDpadDown2.isPressed() && slideServoPosition - 0.05 >= 0.4) {
        slideServoPosition -= 0.05;
      }
      slideLeftServo.setPosition(slideServoPosition);
      slideRightServo.setPosition(slideServoPosition);

      telemetry.addData("forward", gamepad1.left_stick_y);
      telemetry.addData("fun:", gamepad1.left_stick_x);
      telemetry.addData("turn:", gamepad1.right_stick_x);
      telemetry.addData("heading:", botHeading);
      telemetry.addData("rawHeading:", od.getHeading());
      telemetry.addData("slideLeftServo", slideLeftServo.getPosition());
      telemetry.addData("slideRightServo", slideRightServo.getPosition());
      telemetry.addData("wristTurnServo", wristTurnServo.getPosition());
      telemetry.update();
    }
  }
  /*
  private enum ServoState {
      ORIGIN(1),
      RIGHT30(0.8),
      RIGHT60(0.6),
      RIGHT90(0.4);

      public final double slideArmPosition;
      public final double intakeClawPosition;
      //public final double


      ServoState(double armPosition, double intakeClawPosition) {
          this.slideArmPosition = armPosition;
          this.intakeClawPosition = intakeClawPosition;
      }
  }*/
}
