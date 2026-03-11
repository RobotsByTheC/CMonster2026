package frc.robot.data;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.*;

public class LookupTable {
  private static Angle angle = Radians.zero();
  private static AngularVelocity velocity = RadiansPerSecond.zero();

  @SuppressWarnings("unchecked")
  private static <U extends Unit, M extends Measure<U>> Interpolator<M> unitInterpolator() {
    return (startValue, endValue, t) -> (M) endValue.minus(startValue).times(MathUtil.clamp(t, 0, 1)).plus(startValue);
  }

  private static <U extends Unit, M extends Measure<U>> InverseInterpolator<M> inverseUnitInterpolator() {
    return (startValue, endValue, q) -> {
      var totalRange = endValue.minus(startValue);
      if (totalRange.baseUnitMagnitude() <= 0) {
        return 0;
      }

      var queryToStart = q.minus(startValue);
      if (queryToStart.baseUnitMagnitude() <= 0) {
        return 0;
      }

      return ((Dimensionless) queryToStart.div(totalRange)).in(Value);
    };
  }

  public static final InterpolatingTreeMap<Distance, AngularVelocity> SPEED_TABLE = new InterpolatingTreeMap<>(
      inverseUnitInterpolator(), unitInterpolator());
  public static final InterpolatingTreeMap<Distance, Angle> ANGLE_TABLE = new InterpolatingTreeMap<>(
      inverseUnitInterpolator(), unitInterpolator());

  private static void fill(Distance dist, AngularVelocity speed, Angle angle) {
    SPEED_TABLE.put(dist, speed);
    ANGLE_TABLE.put(dist, angle);
  }

  public static void update(Distance distance) {
    velocity = SPEED_TABLE.get(distance);
    angle = ANGLE_TABLE.get(distance);
  }

  public static Angle getAngle() {
    return angle;
  }

  public static AngularVelocity getVelocity() {
    return velocity;
  }

  static {
    fill(Meters.of(1), RPM.of(2000), Degrees.of(5));
    fill(Meters.of(1.1), RPM.of(2100), Degrees.of(7.5));
    fill(Meters.of(1.2), RPM.of(2200), Degrees.of(10));
    fill(Meters.of(1.3), RPM.of(2300), Degrees.of(12.5));
    fill(Meters.of(1.4), RPM.of(2400), Degrees.of(15));
    fill(Meters.of(1.5), RPM.of(2500), Degrees.of(17.5));
    fill(Meters.of(1.6), RPM.of(2600), Degrees.of(20));
    fill(Meters.of(1.7), RPM.of(2700), Degrees.of(22.5));
    fill(Meters.of(1.8), RPM.of(2800), Degrees.of(25));
    fill(Meters.of(1.9), RPM.of(2900), Degrees.of(27.5));
    fill(Meters.of(2), RPM.of(3000), Degrees.of(30));
  }
}
