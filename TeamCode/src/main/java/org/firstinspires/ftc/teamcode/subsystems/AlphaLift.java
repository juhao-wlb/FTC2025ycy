package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Config
public class AlphaLift extends SubsystemBase {
    public static boolean UsePID = false;
    public static double kG = 0;
    public static PIDFController liftPID = new PIDFController(0, 0, 0, 0);
    private DcMotor leftLiftMotor, rightLiftMotor;
    private final HardwareMap hardwareMap;
    public AlphaLift(HardwareMap hm){
        // TODO: Check PID works or not & which motor has encoder
        hardwareMap = hm;
        leftLiftMotor = hardwareMap.get(DcMotor.class,"leftLiftMotor");
        rightLiftMotor = hardwareMap.get(DcMotor.class,"rightLiftMotor");
        leftLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if(UsePID){
            leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    public void setOpenLoop(double vol){
        leftLiftMotor.setPower(vol);
        rightLiftMotor.setPower(vol);
    }
    @Override
    public void periodic(){
        if(UsePID){
            double power = liftPID.calculate(leftLiftMotor.getCurrentPosition()) + kG;
            leftLiftMotor.setPower(power);
            rightLiftMotor.setPower(power);
        }
    }
    public void setSetPoint(double pos){
        liftPID.setSetPoint(pos);
    }
}
