package org.firstinspires.ftc.teamcode.lib.roadrunner.drive;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import lombok.experimental.ExtensionMethod;
import org.firstinspires.ftc.teamcode.lib.GeomUtil;
import org.firstinspires.ftc.teamcode.lib.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants;

@ExtensionMethod({GeomUtil.class})
public class GoBildaLocalizer implements Localizer {
  private final GoBildaPinpointDriver odometry;

  public GoBildaLocalizer(
      final HardwareMap hardwareMap, final DriveConstants.Translation2dHelperClass mountOffsets) {
    odometry = hardwareMap.get(GoBildaPinpointDriver.class, "od");
    odometry.setEncoderDirections(
        DriveConstants.GoBildaXLocalizerDirection, DriveConstants.GoBildaYLocalizerDirection);
    odometry.setEncoderResolution(DriveConstants.GoBildaLocalizerEncoderResolution);
    odometry.setOffsets(mountOffsets.getX(), mountOffsets.getY());
    odometry.resetPosAndIMU();
  }

  public Translation2d getWheelOffsets() {
    return new Translation2d(odometry.getXOffset(), odometry.getYOffset());
  }

  @NonNull
  @Override
  public Pose2d getPoseEstimate() {
    odometry.update();
    return odometry.getPosition().toPose2d();
  }

  public double getHeading() {
    odometry.update();
    return odometry.getHeading();
  }

  @Override
  public void setPoseEstimate(@NonNull Pose2d pose2d) {
    odometry.setPosition(pose2d.toPose2D());
  }

  @Nullable
  @Override
  public Pose2d getPoseVelocity() {
    odometry.update();
    return odometry.getVelocity().toPose2d();
  }

  public double getHeadingVelocity() {
    odometry.update();
    return odometry.getHeadingVelocity();
  }

  /** IMPORTANT: Requires *stationary* robot. */
  public void recalibrateIMU() {
    odometry.recalibrateIMU();
  }

  @Override
  public void update() {}
}
