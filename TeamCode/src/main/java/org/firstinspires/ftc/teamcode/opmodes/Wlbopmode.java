package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;
@TeleOp(name = "Wlbopmode")
public class Wlbopmode extends LinearOpMode {
    public SampleMecanumDrive Wlbdrive;

    @Override
    public void runOpMode() throws InterruptedException {
        Wlbdrive = new SampleMecanumDrive(hardwareMap);
        waitForStart();
        while(opModeIsActive()){
            //Wlbdrive.turn(Math.PI);
            Wlbdrive.setFieldRelativeDrivePower(new Pose2d(-gamepad1.left_stick_y,-gamepad1.left_stick_x,-gamepad1.right_stick_x));
            if(gamepad1.a){
                Wlbdrive.resetHeading();
            }
            telemetry.update();
        }
    }
}
