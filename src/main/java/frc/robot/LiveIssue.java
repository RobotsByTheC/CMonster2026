package frc.robot;

import edu.wpi.first.epilogue.Logged;
import java.util.HashSet;
import java.util.Set;

@Logged
public class LiveIssue {
  public static final LiveIssue INSTANCE = new LiveIssue();

  private final Set<String> ISSUES = new HashSet<>();

  public void addIssue(String issue) {
    ISSUES.add(issue);
  }
  public void clear() {
    ISSUES.clear();
  }
  public Set<String> getIssues() {
    return ISSUES;
  }
}