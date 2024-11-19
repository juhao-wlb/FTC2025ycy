package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;
import java.util.function.BooleanSupplier;

public class StartUntilCommand extends CommandBase {
    private final BooleanSupplier startRunnable2Until;
    private final Runnable runnable1;
    private final Runnable runnable2;
    private boolean timerStarted = false;
    private final ElapsedTime timer = new ElapsedTime();


    /**
    * Execute runnable1 first and execute runnable2 until
    * */
    public StartUntilCommand(BooleanSupplier start2Until, Runnable runnable1, Runnable runnable2) {
        startRunnable2Until = start2Until;
        this.runnable1 = runnable1;
        this.runnable2 = runnable2;
    }

    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public void execute() {
        runnable1.run();

        if (startRunnable2Until.getAsBoolean() && !timerStarted){
            runnable2.run();
            timer.startTime();
            timerStarted = true;
        }

    }

    @Override
    public void end(boolean interrupted) {
        timerStarted = false;
    }

    @Override
    public boolean isFinished() {
        return timer.time(TimeUnit.MILLISECONDS) > 500;
    }

}
