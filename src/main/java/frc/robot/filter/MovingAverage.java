package frc.robot.filter;

import java.util.ArrayList;

public class MovingAverage {
  private final ArrayList<Double> values;
  private final int size;

  /**
   * The MovingAverage class tracks the last {@see size} values given to it, and can be used to
   * average out data.
   */
  public MovingAverage(int size) {
    this.size = size;
    values = new ArrayList<>();
  }

  public void addValue(double value) {
    if (values.size() >= size) values.remove(0);
    values.add(value);
  }

  public double getAverage() {
    double sum = 0;
    for (double d : values) sum += d;
    return sum / values.size();
  }
}
