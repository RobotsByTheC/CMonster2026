package frc.robot.dashboard;

import edu.wpi.first.epilogue.Logged;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

@Logged
public class Dashboard {
  private static final List<DashboardField> DASHBOARD_BOOLEANS = new ArrayList<>();

  public static void addField(DashboardField field) {
    DASHBOARD_BOOLEANS.add(field);
  }
}