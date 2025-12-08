package frc.robot.logging;

import edu.wpi.first.wpilibj.Alert;
import java.util.function.BooleanSupplier;

public class Issue extends Alert {
  private final BooleanSupplier endCondition;

  public Issue(String group, String text, AlertType type, BooleanSupplier endCondition) {
    super(group, text, type);
    this.endCondition = endCondition;
    periodic();
  }

  public void periodic() {
    set(!endCondition.getAsBoolean());
  }

  public boolean isValid() {
    return !endCondition.getAsBoolean();
  }
}