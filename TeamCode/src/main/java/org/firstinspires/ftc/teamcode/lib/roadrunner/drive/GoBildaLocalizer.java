package org.firstinspires.ftc.teamcode.lib.roadrunner.drive;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.lib.GeomUtil;
import org.firstinspires.ftc.teamcode.lib.gobilda.GoBildaPinpointDriver;

import lombok.experimental.ExtensionMethod;

@ExtensionMethod({GeomUtil.class})
public class GoBildaLocalizer implements Localizer {
    private final GoBildaPinpointDriver odometry;

    public GoBildaLocalizer(final HardwareMap hardwareMap) {
        odometry = hardwareMap.get(GoBildaPinpointDriver.class, "od");
        odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odometry.setOffsets(-110,40);
        odometry.resetPosAndIMU();
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        odometry.update();
        return odometry.getPosition().toPose2d();
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

    @Override
    public void update() {}


}
