package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name = "PIDMotorTest")
public class PIDMotorTest extends LinearOpMode {
    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    public static int set_position = 0;

    public static boolean read_only = true;
    public static boolean reverse_1 = false;
    public static boolean reset = true;

    public static String motor_name_1 = "LiftMotor";
    public static double Kp,Ki,Kd;
    public final PIDController motorPID = new PIDController(Kp,Ki,Kd);

    public static DcMotorEx motor1;
    @Override
    public void runOpMode() throws InterruptedException {
        motor1 = hardwareMap.get(DcMotorEx.class,motor_name_1);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        if (reset) {
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (reverse_1) {
            motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        while (opModeIsActive()) {
            if (!read_only) {
                double motorPowerFront = motorPID.calculate(motor1.getCurrentPosition(),set_position);
                motor1.setPower(Range.clip(motorPowerFront, -1, 1));
            }

            telemetry_M.addData("is_busy_1", motor1.isBusy());
//                telemetry_M.addData("encoder_1", motor1.getCurrentPosition());
//                telemetry_M.addData("encoder_2", motor1.getCurrentPosition());

            telemetry_M.addData("position_ticks", motor1.getCurrentPosition());
            telemetry_M.addData("velocity_ticksPerSecond", motor1.getVelocity());
            telemetry_M.update();
        }

    }
}
