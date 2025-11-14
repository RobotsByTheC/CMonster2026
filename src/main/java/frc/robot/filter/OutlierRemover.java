package frc.robot.filter;

import java.util.ArrayList;
import java.util.List;

public class OutlierRemover {
  private final List<Double> VALUES;

  public OutlierRemover() {
    VALUES = new ArrayList<>();
  }

  public double getAverageDistance() {
    double sum = 0;
    for (double distance : VALUES) {
      sum+=distance;
    }
    return sum/VALUES.size();
  }

  public void addValue(double value) {
    double average = getAverageDistance();
    if (VALUES.size() < 10) {
      VALUES.add(value);
      return;
    }
    if (Math.abs((average-value)/average) < 0.5) {
      VALUES.remove(0);
      VALUES.add(value);
    }
  }
}