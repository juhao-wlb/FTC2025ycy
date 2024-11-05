package org.firstinspires.ftc.teamcode.lib.roadrunner.drive;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.lib.gobilda.GoBildaPinpointDriver;

public class GoBildaLocalizer implements Localizer {
    private final GoBildaPinpointDriver odometry;

    public GoBildaLocalizer(final HardwareMap hardwareMap) {
        odometry = hardwareMap.get(GoBildaPinpointDriver.class, "od");
        odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odometry.setOffsets(-100,40);
        odometry.resetPosAndIMU();
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        odometry.update();
        return toPose2d(odometry.getPosition());
    }

    public double getHeading() {
        odometry.update();
        return odometry.getHeading();
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        odometry.setPosition(toPose2D(pose2d));
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        odometry.update();
        return toPose2d(odometry.getVelocity());
    }

    public double getHeadingVelocity() {
        odometry.update();
        return odometry.getHeadingVelocity();
    }

    @Override
    public void update() {}

    public static Pose2d toPose2d(Pose2D pose) {
        return new Pose2d(pose.getX(DistanceUnit.INCH),
                pose.getY(DistanceUnit.INCH),
                pose.getHeading(AngleUnit.RADIANS));
    }

    public static Pose2D toPose2D(Pose2d pose) {
        return new Pose2D(DistanceUnit.INCH, pose.getX(), pose.getY(),
                AngleUnit.RADIANS, pose.getHeading());
    }
}
