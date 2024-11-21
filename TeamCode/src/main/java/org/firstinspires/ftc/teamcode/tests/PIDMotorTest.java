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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp(name = "PIDMotorTest")
public class PIDMotorTest extends LinearOpMode {
    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    public static int set_position = 0;
    public static int set_velocity = 0;
    public static int set_accleration = 0;
    public static int horizontalRotationRads = 0;

    public static boolean read_only = true;
    public static boolean reverse_1 = false;
    public static boolean brake = true;
    public static boolean useFeedforward = false;
    public static GravityMode gravityMode;
    public enum GravityMode {
        COSINE,
        CONSTANT
    }

    public static String motor_name_1 = "liftMotor";
    public static double Kp = 0.0, Ki = 0.0 ,Kd = 0.0;
    public static double kG = 0.0, kS = 0.0, kV = 0.0, kA = 0.0;
    public final PIDController motorPID = new PIDController(Kp,Ki,Kd);

//    void

    public static DcMotorEx motor1;
    @Override
    public void runOpMode() throws InterruptedException {
        double nowPositionTicks, nowVelocityRadPS, lastVelocityRadPS = 0;
        motor1 = hardwareMap.get(DcMotorEx.class, motor_name_1);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();

        if (reverse_1) {
            motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        if (brake) {
            motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        while (opModeIsActive()) {

            nowPositionTicks = motor1.getCurrentPosition();
            nowVelocityRadPS = motor1.getVelocity(AngleUnit.RADIANS);

            if (!read_only) {
                double motorPowerFront = motorPID.calculate(nowPositionTicks,set_position);
                double Feedforward = 0;
                if(useFeedforward){
                    Feedforward = Math.copySign(kS, nowVelocityRadPS) + kV * set_velocity + kA * set_accleration + (gravityMode==GravityMode.CONSTANT? kG : kG * Math.cos(nowPositionTicks / 12 + horizontalRotationRads));
                }
                motorPowerFront+=Feedforward;
                motor1.setPower(Range.clip(motorPowerFront, -1, 1));
            }



            telemetry_M.addData("is_busy_1", motor1.isBusy());
//                telemetry_M.addData("encoder_1", motor1.getCurrentPosition());
//                telemetry_M.addData("encoder_2", motor1.getCurrentPosition());

            telemetry_M.addData("position_ticks", nowPositionTicks);
            telemetry_M.addData("velocity_RadsPerSecond", nowVelocityRadPS);
            telemetry_M.update();
        }

    }
}
