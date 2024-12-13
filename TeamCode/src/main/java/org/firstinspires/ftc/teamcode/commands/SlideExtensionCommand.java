package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.concurrent.TimeUnit;

public class SlideExtensionCommand extends CommandBase {
  private final Runnable runnable;
  private final ElapsedTime timer;
  private double lastTime = 0;

  public SlideExtensionCommand(Runnable runnable) {
    this.runnable = runnable;
    timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
  }

  @Override
  public void initialize() {
    lastTime = 0;
    timer.startTime();
  }

  @Override
  public void execute() {
    if (timer.time(TimeUnit.MILLISECONDS) - lastTime > 200) {
      lastTime = timer.time(TimeUnit.MILLISECONDS);
      runnable.run();
    }
  }

  @Override
  public void end(boolean interrupted) {
    timer.reset();
    lastTime = 0;
  }
}
