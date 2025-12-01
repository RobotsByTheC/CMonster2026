package frc.robot.logging;

import edu.wpi.first.wpilibj.Alert;
import java.util.function.BooleanSupplier;

public class Issue extends Alert {
  private final BooleanSupplier END_CONDITION;

  public Issue(String group, String text, AlertType type, BooleanSupplier endCondition) {
    super(group, text, type);
    END_CONDITION = endCondition;
    periodic();
  }

  public void periodic() {
    set(!END_CONDITION.getAsBoolean());
  }

  public boolean isValid() {
    return !END_CONDITION.getAsBoolean();
  }
}