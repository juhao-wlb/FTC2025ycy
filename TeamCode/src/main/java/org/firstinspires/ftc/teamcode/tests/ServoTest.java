package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "servo test")
@Config
public class ServoTest extends LinearOpMode {

    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    public static boolean read_only = false;
    public static boolean reverse = false;
    public static double servo_pos1 = 1;

    public static String servo_name1 = "clawTurnServo";
    public static String servo_encoder_name = "";
    public static String servo_name2 = "doorRight";
    private Servo servo0 = null;
    private AnalogInput encoder0 = null;
    private Servo servo1 = null;

    private boolean prevVal = false;
    private boolean shouldStop = false;

    @Override
    public void runOpMode() {

        servo0 = hardwareMap.get(Servo.class, servo_name1);
        if(servo_encoder_name!=""){
            encoder0 = hardwareMap.get(AnalogInput.class, servo_encoder_name);
        }
        //servo1 = hardwareMap.get(Servo.class, servo_name2);
        //upperMagnetic = hardwareMap.get(TouchSensor.class, "upperMagnetic");
        if (reverse){
            servo0.setDirection(Servo.Direction.REVERSE);
        }
        waitForStart();
        while (opModeIsActive()) {

            if(servo_encoder_name!=""){
                telemetry_M.addData("EncoderPosition (Voltage)", encoder0.getVoltage());
                telemetry_M.addData("AnalogMaxVoltage", encoder0.getMaxVoltage());
            }

            if (!read_only) {
                servo0.setPosition(servo_pos1);
                //servo1.setPosition(servo_pos1);
                //servo1.setPosition(servo_pos1);
//                servo1.setPosition(servo_pos2);
                telemetry_M.addData(servo_name1, servo0.getPosition());
//                telemetry_M.addData("rightfront", servo1.getPosition());

            }
            else {
                servo0.setPosition(0.5);
                //servo1.setPosition(0.5);
            }

            //telemetry_M.addData("Magnetic Triggered", upperMagnetic.isPressed());
            telemetry_M.update();
        }
    }
}
