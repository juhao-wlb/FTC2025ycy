package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.lib.Units;

/*
 * Constants shared between multiple drive types.
 *
 * TODO: Tune or adjust the following constants to fit your robot. Note that the non-final
 * fields may also be edited through the dashboard (connect to the robot's WiFi network and
 * navigate to https://192.168.49.1:8080/dash). Make sure to save the values here after you
 * adjust them in the dashboard; **config variable changes don't persist between app restarts**.
 *
 * These are not the only parameters; some are located in the localizer classes, drive base classes,
 * and op modes themselves.
 */
@Config
public class DriveConstants {
  public static final RobotType currentRobot = RobotType.ALPHA;

  public enum RobotType {
    ALPHA,
    BETA
  }

  public static final Pose2d xPose =
      currentRobot == RobotType.ALPHA
          ? new Pose2d(Units.mmToInches(-25), Units.mmToInches(155), Units.degreesToRadians(0))
          : new Pose2d(Units.mmToInches(0), Units.mmToInches(114.5), Units.degreesToRadians(0));

  public static final Pose2d yPose =
      currentRobot == RobotType.ALPHA
          ? new Pose2d(Units.mmToInches(-25), Units.mmToInches(-155), Units.degreesToRadians(90))
          : new Pose2d(Units.mmToInches(-98), Units.mmToInches(15), Units.degreesToRadians(90));

  /*
   * These are motor constants that should be listed online for your motors.
   */
  public static final double TICKS_PER_REV =
      384.5; // From https://learnroadrunner.com/drive-constants.html#ticks-per-rev-max-rpm ->
  // https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-13-7-1-ratio-24mm-length-8mm-rex-shaft-435-rpm-3-3-5v-encoder/
  public static final double MAX_RPM = 435; // From
  // https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-13-7-1-ratio-24mm-length-8mm-rex-shaft-435-rpm-3-3-5v-encoder/

  // DO NOT TOUCH THESE VALUES ref.
  // https://learnroadrunner.com/drive-constants.html#run-using-encoder-motor-velo-pid
  public static final boolean RUN_USING_ENCODER = false;
  public static PIDFCoefficients MOTOR_VELO_PID =
      new PIDFCoefficients(0, 0, 0, getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));

  /*
   * These are physical constants that can be determined from your robot (including the track
   * width; it will be tune empirically later although a rough estimate is important). Users are
   * free to chose whichever linear distance unit they would like so long as it is consistently
   * used. The default values were selected with inches in mind. Road runner uses radians for
   * angular distances although most angular parameters are wrapped in Math.toRadians() for
   * convenience. Make sure to exclude any gear ratio included in MOTOR_CONFIG from GEAR_RATIO.
   */
  public static double WHEEL_RADIUS = 2; // INCH!!
  public static double GEAR_RATIO =
      1; // output (wheel) speed / input (motor) speed, >1 -> 加速 <1 -> 减速
  public static double TRACK_WIDTH = 13.19; // INCH!!

  /*
   * These are the feedforward parameters used to model the drive motor behavior. If you are using
   * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
   * motor encoders or have elected not to use them for velocity control, these values should be
   * empirically tuned.
   */
  public static double kV =
      currentRobot == RobotType.ALPHA ? 0.00955 : 1.0 / rpmToVelocity(MAX_RPM);
  public static double kA = currentRobot == RobotType.ALPHA ? 0.002 : 0;
  public static double kStatic = currentRobot == RobotType.ALPHA ? 0.05 : 0;

  /*
   * These values are used to generate the trajectories for you robot. To ensure proper operation,
   * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
   * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
   * small and gradually increase them later after everything is working. All distance units are
   * inches.
   */
  public static double MAX_VEL = currentRobot == RobotType.ALPHA ? 87.53357957701085 : 64;
  public static double MAX_ACCEL = 64;
  public static double MAX_ANG_VEL =
      Math.toRadians(currentRobot == RobotType.ALPHA ? 143.8430517241895 : 60);
  public static double MAX_ANG_ACCEL = Math.toRadians(60);

  /*
   * Adjust the orientations here to match your robot. See the FTC SDK documentation for details.
   */
  public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR =
      RevHubOrientationOnRobot.LogoFacingDirection.UP;
  public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR =
      RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

  public static double encoderTicksToInches(double ticks) {
    return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
  }

  public static double rpmToVelocity(double rpm) {
    return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
  }

  public static double getMotorVelocityF(double ticksPerSecond) {
    // see
    // https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
    return 32767 / ticksPerSecond;
  }
}
