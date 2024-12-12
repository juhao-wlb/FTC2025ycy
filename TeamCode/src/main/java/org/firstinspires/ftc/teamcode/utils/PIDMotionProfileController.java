package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.controller.PIDFController;

public class PIDMotionProfileController extends PIDFController {
  public double kG = 0.0, kS = 0.0, kV = 0.0, kA = 0.0;
  public double MAX_VELOCITY, MAX_ACCLERATION;
  public double TicksPerRev = 1024;
  private double set_velocity, set_accleration;

  public enum GravityMode {
    COSINE,
    CONSTANT
  }

  public GravityMode gravityMode = GravityMode.CONSTANT;

  public PIDMotionProfileController(
      double kp, double ki, double kd, double kS, double kV, double kA, double kG) {
    super(kp, ki, kd, 0);
    this.kG = kG;
    this.kS = kS;
    this.kV = kV;
    this.kA = kA;
  }

  public PIDMotionProfileController(double kp, double ki, double kd) {
    this(kp, ki, kd, 0, 0, 0, 0);
  }

  public PIDMotionProfileController withKp(double kP) {
    setP(kP);
    return this;
  }

  public PIDMotionProfileController withKi(double kI) {
    setI(kI);
    return this;
  }

  public PIDMotionProfileController withKd(double kD) {
    setD(kD);
    return this;
  }

  public PIDMotionProfileController withKs(double kS) {
    this.kS = kS;
    return this;
  }

  public PIDMotionProfileController withKv(double kV) {
    this.kV = kV;
    return this;
  }

  public PIDMotionProfileController withKa(double kA) {
    this.kA = kA;
    return this;
  }

  public PIDMotionProfileController withKg(double kG) {
    this.kG = kG;
    return this;
  }

  /** The extended constructor. */
  public PIDMotionProfileController(double kp, double ki, double kd, double sp, double pv) {
    super(kp, ki, kd, 0, sp, pv);
  }

  public void setPID(double kp, double ki, double kd) {
    setPIDF(kp, ki, kd, 0);
  }

  public void setPIDSVA(double kp, double ki, double kd, double kS, double kV, double kA) {
    setPIDF(kp, ki, kd, 0);
    this.kS = kS;
    this.kV = kV;
    this.kA = kA;
  }

  @Override
  public double calculate(double pv) {
    double PIDPower = super.calculate(pv);
    double feedforward =
        Math.copySign(kS, PIDPower)
            + kV * set_velocity
            + kA * set_accleration
            + (gravityMode == GravityMode.CONSTANT
                ? kG
                : kG * Math.cos(pv / TicksPerRev * Math.PI * 2));
    return PIDPower + feedforward;
  }

  public PIDMotionProfileController copy() {
    PIDMotionProfileController controller =
        new PIDMotionProfileController(getP(), getI(), getD(), kS, kV, kA, kG);
    controller.gravityMode = gravityMode;
    controller.TicksPerRev = TicksPerRev;
    return controller;
  }
}
