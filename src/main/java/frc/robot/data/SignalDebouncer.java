package frc.robot.data;

import java.util.ArrayList;
import java.util.List;

public class SignalDebouncer {
  private final boolean desiredValue;
  private final int requiredTicks;
  private final List<Boolean> history;

  public SignalDebouncer(boolean desiredValue, int requiredTicks) {
    this.desiredValue = desiredValue;
    this.requiredTicks = requiredTicks;
    history = new ArrayList<>();
  }

  public void addValue(boolean value) {
    history.add(value);

    if (history.size() > requiredTicks) {
      history.remove(0);
    }
  }

  public boolean getResult() {
    for (Boolean value : history) {
      if (value != desiredValue) {
        return false;
      }
    }
    return true;
  }
}