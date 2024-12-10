package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrive;

public class AutoAlignCommand extends CommandBase {
  private final MecanumDrive drive;
  private final Vision limelight;
  private final ElapsedTime timer;

  // PID Controllers
  private final PIDController strafePID;
  private final PIDController distancePID;

  // Camera and Target Constants
  private static final double DESIRED_DISTANCE = 24.0;

  // Distance Safety Bounds (in mm)
  private static final double MIN_SAFE_DISTANCE = 50.0;
  private static final double MAX_SAFE_DISTANCE = 600.0;

  // Rate Limiting
  private static final double MAX_DISTANCE_CHANGE_PER_SEC = 12.0;
  private static final double MAX_STRAFE_CHANGE_PER_SEC = 0.3;
  private double lastDistanceOutput = 0.0;
  private double lastStrafeOutput = 0.0;
  private double lastUpdateTime = 0.0;

  // Distance Ramping
  private static final double DISTANCE_RAMP_FACTOR = 0.8;
  private static final double MIN_DISTANCE_POWER = 0.1;
  private static final double MAX_DISTANCE_POWER = 0.8;

  // Alignment tolerances
  private static final double STRAFE_TOLERANCE = 60.0;
  private static final double DISTANCE_TOLERANCE = 60.0;

  // Timeout
  private static final double TIMEOUT_SECONDS = 3.0;

  public AutoAlignCommand(MecanumDrive drive, Vision limelight) {
    this.drive = drive;
    this.limelight = limelight;
    this.timer = new ElapsedTime();

    // Initialize PID Controllers
    this.strafePID = new PIDController(0.03, 0.0001, 0.0002);
    this.distancePID = new PIDController(0.04, 0.0001, 0.0002);

    // Set PID constraints
    strafePID.setTolerance(STRAFE_TOLERANCE);
    distancePID.setTolerance(DISTANCE_TOLERANCE);
    strafePID.setIntegrationBounds(-0.3, 0.3);
    distancePID.setIntegrationBounds(-0.3, 0.3);

    addRequirements(drive); // Command requires exclusive access to drive
  }

  @Override
  public void initialize() {
    timer.reset();
    lastUpdateTime = timer.seconds();
    lastDistanceOutput = 0.0;
    lastStrafeOutput = 0.0;
  }

  @Override
  public void execute() {
    double currentTime = timer.seconds();
    double deltaTime = currentTime - lastUpdateTime;

    double tx = limelight.getTx(0.0);
    boolean targetVisible = limelight.isTargetVisible();

    if (targetVisible) {
      double currentDistance = limelight.getDistance();

      // Safety bounds check
      if (currentDistance < MIN_SAFE_DISTANCE || currentDistance > MAX_SAFE_DISTANCE) {
        drive.stop();
        return;
      }

      // Calculate PID outputs
      double strafeOutput = strafePID.calculate(tx, 0);
      double distanceOutput = distancePID.calculate(currentDistance, DESIRED_DISTANCE);

      // Apply rate limiting
      strafeOutput =
          applyRateLimiting(strafeOutput, lastStrafeOutput, MAX_STRAFE_CHANGE_PER_SEC, deltaTime);
      distanceOutput =
          applyRateLimiting(
              distanceOutput, lastDistanceOutput, MAX_DISTANCE_CHANGE_PER_SEC, deltaTime);

      // Apply distance-based power ramping
      distanceOutput = applyDistanceRamping(distanceOutput, currentDistance);

      // Drive the robot
      drive.moveRobot(distanceOutput, strafeOutput, 0); // No rotation

      // Store values for next iteration
      lastStrafeOutput = strafeOutput;
      lastDistanceOutput = distanceOutput;
    } else {
      drive.stop();
    }

    lastUpdateTime = currentTime;
  }

  @Override
  public boolean isFinished() {
    // Check timeout
    //    if (timer.seconds() > TIMEOUT_SECONDS) {
    //      return true;
    //    }

    // Check if target is visible
    boolean targetVisible = limelight.isTargetVisible();

    if (!targetVisible) {
      return false;
    }

    if (limelight.isDataOld()) {
      return true;
    }

    // Get current measurements
    double tx = limelight.getTx(0.0);
    double currentDistance = limelight.getDistance();

    // Check if aligned and at correct distance
    boolean isAligned = Math.abs(tx) < STRAFE_TOLERANCE;
    boolean isAtDistance = Math.abs(currentDistance - DESIRED_DISTANCE) < DISTANCE_TOLERANCE;

    return isAligned && isAtDistance;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  private double applyRateLimiting(
      double desiredOutput, double lastOutput, double maxChangePerSec, double deltaTime) {
    double maxChange = maxChangePerSec * deltaTime;
    double change = desiredOutput - lastOutput;
    change = Math.max(Math.min(change, maxChange), -maxChange);
    return lastOutput + change;
  }

  private double applyDistanceRamping(double output, double currentDistance) {
    double distanceRatio =
        (currentDistance - MIN_SAFE_DISTANCE) / (MAX_SAFE_DISTANCE - MIN_SAFE_DISTANCE);
    double rampedPower =
        output * (DISTANCE_RAMP_FACTOR * distanceRatio + (1 - DISTANCE_RAMP_FACTOR));
    rampedPower = Math.max(MIN_DISTANCE_POWER, Math.min(MAX_DISTANCE_POWER, rampedPower));
    return Math.signum(output) * rampedPower;
  }
}
