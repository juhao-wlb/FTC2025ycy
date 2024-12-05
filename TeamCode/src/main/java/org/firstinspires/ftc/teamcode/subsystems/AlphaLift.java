package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.PIDMotionProfileController;

import java.util.Set;

@Config
public class AlphaLift extends SubsystemBase {
    public static boolean UsePID = true;
    public static double TicksPerRev = 567;//((((1+(46/17))) * (1+(46/17))) * 28);
    public static double SpoolInnerRadiusMM = 36.25000/2;
    public static double SetpointMM = -1;
    public static PIDMotionProfileController leftLiftPID = new PIDMotionProfileController(0.0075, 0, 0, 0.015, 0, 0, 0.05);
    public static PIDMotionProfileController rightLiftPID = leftLiftPID.copy();
    private DcMotor leftLiftMotor, rightLiftMotor;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry =FtcDashboard.getInstance().getTelemetry();
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
            leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        leftLiftPID.setTolerance(20, 10);
        rightLiftPID.setTolerance(20, 10);
    }
    public void setOpenLoop(double vol){
        leftLiftMotor.setPower(vol);
        rightLiftMotor.setPower(vol);
    }
    @Override
    public void periodic(){
        leftLiftPID.TicksPerRev = TicksPerRev;
//        rightLiftPID.TicksPerRev = TicksPerRev;
        if(SetpointMM!=-1){
            setSetPoint(SetpointMM);
        }
        if(UsePID){
            telemetry.addData("setpoint", leftLiftPID.getSetPoint());
            telemetry.addData("leftLiftMotorPos", leftLiftMotor.getCurrentPosition());
            telemetry.addData("rightLiftMotorPos", rightLiftMotor.getCurrentPosition());
            double power = leftLiftPID.calculate(leftLiftMotor.getCurrentPosition());
            telemetry.addData("leftLiftMotorPower", power);
            leftLiftMotor.setPower(power);
//            power = rightLiftPID.calculate(rightLiftMotor.getCurrentPosition());
//            telemetry.addData("rightLiftMotorPower", power);
//            rightLiftMotor.setPower(power);
        }
        telemetry.update();
    }
    public void setSetPointByMotorRot(double rev){
        leftLiftPID.setSetPoint(rev*TicksPerRev);
        rightLiftPID.setSetPoint(rev*TicksPerRev);
    }
    public double getPositionByMotorRot(){
        return leftLiftMotor.getCurrentPosition()/*+rightLiftMotor.getCurrentPosition())*0.5*//TicksPerRev;
    }
    public void setSetPoint(double posmm){
        setSetPointByMotorRot(posmm/(2*Math.PI*SpoolInnerRadiusMM));
    }
    public double getPositionMM(){
        return getPositionByMotorRot()*(2*Math.PI*SpoolInnerRadiusMM);
    }
    public boolean atSetPoint(){
        return leftLiftPID.atSetPoint();
    }
}
