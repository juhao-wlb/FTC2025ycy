package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.function.Supplier;

@Config
public class AlphaClaw extends SubsystemBase {
    public static double slideServo_ExtendPose = 0.73;
    public static double slideServo_ContractPose = 0.17;
    public static double clawServo_OpenPose = 0.4;
    public static double clawServo_GrabPose = 0.66;
    public static double clawTurnServo_MinPose = 0;
    public static double clawTurnServo_MaxPose = 1;
    private final HardwareMap hardwareMap;
    private final Servo clawServo, clawTurnServo, slideServo;
    private double clawTurnServoSetpoint = clawTurnServo_MinPose;
    public AlphaClaw(HardwareMap hm){
        hardwareMap = hm;
        clawServo = hardwareMap.get(Servo.class,"clawServo");
        clawTurnServo = hardwareMap.get(Servo.class,"clawTurnServo");
        slideServo = hardwareMap.get(Servo.class,"slideServo");
        slideServo.setDirection(Servo.Direction.FORWARD);
        clawTurnServo.setDirection(Servo.Direction.REVERSE);
        setDefaultYaw();
    }
    public void aim(Pose2d blockPose){
        // TODO: complete the code
    }

    public void aim(double slideSetpoint, double yawSetpointRot){
        // TODO: calc setpoint based on joint, to make it linear
        slideServo.setPosition(slideSetpoint*(slideServo_ExtendPose - slideServo_ContractPose)+slideServo_ExtendPose);
        clawServo.setPosition(clawServo_OpenPose);
        setYaw(yawSetpointRot);
    }
    public void grab(){
        clawServo.setPosition(clawServo_GrabPose);
    }
    public void retract(){
        slideServo.setPosition(slideServo_ContractPose);
        setDefaultYaw();
    }
    public void release(){
        clawServo.setPosition(clawServo_OpenPose);
    }
    public void setYaw(double yawSetpointRot){
        yawSetpointRot *= 2;
        yawSetpointRot += 0.5;
        clawTurnServoSetpoint = yawSetpointRot - Math.floor(yawSetpointRot);
        updateClawTurnServo();
    }
    public void setDefaultYaw(){
        clawTurnServoSetpoint = (clawTurnServo_MaxPose + clawTurnServo_MinPose) / 2;
        updateClawTurnServo();
    }
    private void updateClawTurnServo(){
        clawTurnServo.setPosition(clawTurnServoSetpoint);
    }
    public Command aimCommand(Supplier<Pose2d> getBlockPos){
        return new InstantCommand(() -> aim(getBlockPos.get()));
    }
    public Command grabCommand(){
        return new InstantCommand(this::grab);
    }
    public Command retractCommand(){
        return new InstantCommand(this::retract);
    }
}