package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.utils.MathUtils;

import lombok.Getter;
import lombok.Setter;

public class Lift extends SubsystemBase {
    private final double kP = 0.02, kI = 0.0, kD = 0.0, kG = 0.03;
    private final PIDController pidController;
    private final DcMotor liftMotor;
    private double setpointTicks = 0.0;

    @Getter @Setter private Goal goal = Goal.STOW;

    public Lift (final HardwareMap hardwareMap) {
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pidController = new PIDController(kP, kI, kD);
    }

    public void runSetpoint(double ticks) {
        setpointTicks = Range.clip(ticks, 0.0, 1500.0);
    }

    public void runLiftOpen(double percent) {
        goal = Goal.OPEN_LOOP;
        liftMotor.setPower(Range.clip(percent, -1, 1));
    }

    public double getCurrentPosition () {
        return liftMotor.getCurrentPosition();
    }

    public boolean atGoal() {
        return MathUtils.isNear(goal.setpointTicks, liftMotor.getCurrentPosition(), 5);
    }

    @Override
    public void periodic() {
        if (goal == Goal.OPEN_LOOP) return;

        setpointTicks = goal.setpointTicks;
        double pidPower = pidController.calculate(liftMotor.getCurrentPosition(), setpointTicks);
        liftMotor.setPower(Range.clip(pidPower + kG, -1, 1));
    }

    public enum Goal {
        BASKET(1500.0),
        STOW(0.0),
        PRE_HANG(0.0),
        OPEN_LOOP(0.0);

        private final double setpointTicks;
        Goal(double setpointTicks) {
            this.setpointTicks = setpointTicks;
        }
    }

}
