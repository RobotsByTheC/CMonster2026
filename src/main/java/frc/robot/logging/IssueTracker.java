package frc.robot.logging;

import edu.wpi.first.wpilibj.Alert;
import java.util.ArrayList;
import java.util.List;

public class IssueTracker {
  public static final List<Issue> ALERTS = new ArrayList<>();
  private static boolean hasIssue = false;
  private static final Issue NO_ISSUES = new Issue("IssueTracker", "No issues present", Alert.AlertType.kInfo, () -> hasIssue);

  public static void periodicUpdate() {
    hasIssue = false;
    for (Issue issue : ALERTS) {
      issue.periodic();
      if (issue.isValid()) hasIssue = true;
    }
    NO_ISSUES.periodic();
  }

  public static void addIssue(Issue issue) {
    ALERTS.add(issue);
  }
}