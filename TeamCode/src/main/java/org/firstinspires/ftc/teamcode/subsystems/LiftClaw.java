package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LiftClaw extends SubsystemBase {
    private final Servo liftArmServo;
    private final Servo liftClawServo;
    private boolean isClawOpen;

    public LiftClaw(final HardwareMap hardwareMap) {
        liftArmServo = hardwareMap.get(Servo.class, "liftArmServo"); // 0.3 Up 0.7 Down
        liftClawServo = hardwareMap.get(Servo.class, "clawServo"); // 0 Close 0.5 Open
    }

    public void switchLiftClaw() {
        if(isClawOpen) {
            liftClawServo.setPosition(0.25);
        }
        else {
            liftClawServo.setPosition(0.5);
        }
        isClawOpen = !isClawOpen;
    }

    public void openClaw() {
        liftClawServo.setPosition(0.5);
    }

    public void closeClaw() {
        liftClawServo.setPosition(0.25);
    }

    public void upLiftArm () {
        liftArmServo.setPosition(0.725);
    }

    public void foldLiftArm() {
        liftArmServo.setPosition(0.2);
    }

}
