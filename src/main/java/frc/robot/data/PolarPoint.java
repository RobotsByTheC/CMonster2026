package frc.robot.data;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public record PolarPoint(Distance distance, Angle angle) {
  public Distance getX() {
    return distance.times(Math.cos(angle.in(Radians)));
  }
  public Distance getY() {
    return distance.times(Math.sin(angle.in(Radians)));
  }
}
