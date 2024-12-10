package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import lombok.Getter;
import lombok.Setter;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.utils.MathUtils;

public class Lift extends SubsystemBase {
  private final double kP = 0.02, kI = 0.0, kD = 0.0, kG = 0.0;
  private final PIDController pidController;
  private final DcMotorEx liftMotor;
  private double setpointTicks = 0.0;
  private MultipleTelemetry telemetry;

  @Getter @Setter private Goal goal = Goal.STOW;

  public Lift(final HardwareMap hardwareMap, Telemetry telemetry) {
    liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
    liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    pidController = new PIDController(kP, kI, kD);

    this.telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
  }

  public void runSetpoint(double ticks) {
    setpointTicks = Range.clip(ticks, 0.0, 1500.0);
  }

  public void runLiftOpen(double percent) {
    goal = Goal.OPEN_LOOP;
    liftMotor.setPower(Range.clip(percent, -1, 1));
  }

  public Command resetCommand() {
    return new StartEndCommand(
        () -> {
          runLiftOpen(-0.6);
        },
        () -> {
          pidController.reset();
          pidController.calculate(0);
          runLiftOpen(0);
          liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
          goal = Goal.STOW;
          telemetry.addData("Lift Current Position", liftMotor.getCurrentPosition());
          telemetry.addData("Error", pidController.getPositionError());
          telemetry.update();
        },
        this);
  }

  public double getCurrentPosition() {
    return liftMotor.getCurrentPosition();
  }

  public boolean atGoal() {
    return MathUtils.isNear(goal.setpointTicks, liftMotor.getCurrentPosition(), 5);
  }

  public boolean atHome() {
    return MathUtils.isNear(Goal.STOW.setpointTicks, liftMotor.getCurrentPosition(), 5);
  }

  public boolean atPreHang() {
    return MathUtils.isNear(Goal.PRE_HANG.setpointTicks, liftMotor.getCurrentPosition(), 5);
  }

  @Override
  public void periodic() {
    //
    //        telemetry.addData("Lift Current Position", liftMotor.getCurrentPosition());
    //        telemetry.addData("Error", pidController.getPositionError());
    //        telemetry.update();
    if (goal == Goal.OPEN_LOOP) return;

    liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    setpointTicks = goal.setpointTicks;
    double pidPower = pidController.calculate(liftMotor.getCurrentPosition(), setpointTicks);
    liftMotor.setPower(Range.clip(pidPower + kG, -1, 1));

    telemetry.addData("Lift Current Power", liftMotor.getPower());
    telemetry.addData("Lift Goal", goal);
    telemetry.addData("Lift At Home", atHome());
    telemetry.addData("controller", pidController.atSetPoint());
    telemetry.addData("Lift Current Position", liftMotor.getCurrentPosition());
    telemetry.addData("Error", pidController.getPositionError());

    telemetry.addData("Current", liftMotor.getCurrent(CurrentUnit.AMPS));
    telemetry.update();
  }

  public enum Goal {
    BASKET(1500.0),
    STOW(0.0),
    PRE_HANG(300.0),
    HANG(0),
    OPEN_LOOP(0.0);

    private final double setpointTicks;

    Goal(double setpointTicks) {
      this.setpointTicks = setpointTicks;
    }
  }
}
