package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;
import java.util.concurrent.TimeUnit;
import java.util.function.LongSupplier;

public class WaitLambdaCommand extends CommandBase {

  protected Timing.Timer m_timer;

  /**
   * Creates a new WaitCommand. This command will do nothing, and end after the specified duration.
   *
   * @param millis the time to wait, in milliseconds
   */
  public WaitLambdaCommand(LongSupplier millis) {
    m_timer = new Timing.Timer(millis.getAsLong(), TimeUnit.MILLISECONDS);
    setName(m_name + ": " + millis + " milliseconds");
  }

  @Override
  public void initialize() {
    m_timer.start();
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.pause();
  }

  @Override
  public boolean isFinished() {
    return m_timer.done();
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
