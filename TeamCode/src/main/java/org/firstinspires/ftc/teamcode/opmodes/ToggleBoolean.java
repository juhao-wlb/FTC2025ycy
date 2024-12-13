package org.firstinspires.ftc.teamcode.opmodes;

import java.util.function.BooleanSupplier;

public class ToggleBoolean {
  private final BooleanSupplier buttonValue;
  private boolean previousButtonValue = false;

  public ToggleBoolean(BooleanSupplier supplier) {
    buttonValue = supplier;
  }

  public boolean isPressed() {
    boolean res = !previousButtonValue && buttonValue.getAsBoolean();
    previousButtonValue = buttonValue.getAsBoolean();
    return res;
  }

  public boolean getLastButton() {
    return previousButtonValue;
  }
}
