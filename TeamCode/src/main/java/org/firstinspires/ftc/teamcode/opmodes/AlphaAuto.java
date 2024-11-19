package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;

@Autonomous(name = "AlphaAuto")
public class AlphaAuto extends LinearOpMode {
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    Pose2d startPoint = new Pose2d(0,0);
    Trajectory tra1 = drive.trajectoryBuilder(startPoint)
            .lineToConstantHeading(new Vector2d(10,0)).build();
    Trajectory tra2 = drive.trajectoryBuilder(tra1.end())
            .lineToConstantHeading(new Vector2d(0,0)).build();




    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        drive.followTrajectory(tra1);
        drive.followTrajectory(tra2);

        while(opModeIsActive() && !isStopRequested()) {

        }



    }
}
