package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;

@TeleOp(name = "pinPointTestTeleOP")
@Config
public class pinpointLocalizerTest extends LinearOpMode {
  //    private GoBildaLocalizer odometry;
  private SampleMecanumDrive drive;
  public static boolean test;

  @Override
  public void runOpMode() throws InterruptedException {

    //        odometry = new GoBildaLocalizer(hardwareMap, new Pose2d(-100, 40));
    drive = new SampleMecanumDrive(hardwareMap);
    waitForStart();
    while (opModeIsActive()) {
      drive.setFieldRelativeDrivePower(
          new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x));
      telemetry.addData("heading:", drive.getHeading());
      telemetry.addData("positions:", drive.getPoseEstimate());
      telemetry.addData("velocities:", drive.getPoseVelocity());
      if (gamepad1.a) {
        drive.resetHeading();
        //                odometry.recalibrateIMU();
      }
      telemetry.update();
    }
  }
}
//
// package org.firstinspires.ftc.teamcode.lib.roadrunner.drive.opmode;
//
// import com.acmerobotics.roadrunner.geometry.Pose2d;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.hardware.DcMotor;
//
// import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;
//
/// **
// * This is a simple teleop routine for testing localization. Drive the robot around like a normal
// * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
// * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
// * exercise is to ascertain whether the localizer has been configured properly (note: the pure
// * encoder localizer heading may be significantly off if the track width has not been tuned).
// */
// @TeleOp(group = "drive")
// public class LocalizationTest extends LinearOpMode {
//    @Override
//    public void runOpMode() throws InterruptedException {
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//
//        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        waitForStart();
//
//        while (!isStopRequested()) {
//            drive.setFieldRelativeDrivePower(
//                    new Pose2d(
//                            -gamepad1.left_stick_y,
//                            -gamepad1.left_stick_x,
//                            -gamepad1.right_stick_x
//                    )
//            );
//            if(gamepad1.dpad_left) {
//                drive.resetHeading();
//            }
//
//            drive.update();
//
//            Pose2d poseEstimate = drive.getPoseEstimate();
//            telemetry.addData("x", poseEstimate.getX());
//            telemetry.addData("y", poseEstimate.getY());
//            telemetry.addData("heading", poseEstimate.getHeading());
//            telemetry.update();
//        }
//    }
// }
