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
    fill(Meters.of(0), RPM.of(0), Degrees.of(0));
    fill(Meters.of(1), RPM.of(500), Degrees.of(2.5));
    fill(Meters.of(2), RPM.of(1000), Degrees.of(5));
    fill(Meters.of(3), RPM.of(1800), Degrees.of(7.5));
    fill(Meters.of(4), RPM.of(2400), Degrees.of(10));
    fill(Meters.of(5), RPM.of(2800), Degrees.of(12.5));
  }
}
