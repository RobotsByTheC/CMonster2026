package frc.robot.subsystems.shooter;

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

import java.util.Comparator;

import static edu.wpi.first.units.Units.*;

public record LookupTable() {
  public static <U extends Unit, M extends Measure<U>> Interpolator<M> unitInterpolator() {
    return (startValue, endValue, t) -> {
      // ((end - start) * t) + start
      return (M) endValue.minus(startValue).times(MathUtil.clamp(t, 0, 1)).plus(startValue);
    };
  }

  public static <U extends Unit, M extends Measure<U>> InverseInterpolator<M> inverseUnitInterpolator() {
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

	public static final InterpolatingTreeMap<Distance, AngularVelocity> SPEED_TABLE = new InterpolatingTreeMap<>(inverseUnitInterpolator(), unitInterpolator());
  public static final InterpolatingTreeMap<Distance, Angle> ANGLE_TABLE = new InterpolatingTreeMap<>(inverseUnitInterpolator(), unitInterpolator());

  private static void fill(Distance dist, AngularVelocity speed, Angle angle) {
    SPEED_TABLE.put(dist, speed);
    ANGLE_TABLE.put(dist, angle);
  }

  static {
    fill(Meters.of(1), RPM.of(1000), Degrees.of(80));
    fill(Meters.of(1.1), RPM.of(1100), Degrees.of(75));
    fill(Meters.of(1.2), RPM.of(1200), Degrees.of(70));
    fill(Meters.of(1.3), RPM.of(1300), Degrees.of(65));
    fill(Meters.of(1.4), RPM.of(1400), Degrees.of(60));
    fill(Meters.of(1.5), RPM.of(1500), Degrees.of(55));
    fill(Meters.of(1.6), RPM.of(1600), Degrees.of(50));
    fill(Meters.of(1.7), RPM.of(1700), Degrees.of(45));
    fill(Meters.of(1.8), RPM.of(1800), Degrees.of(40));
    fill(Meters.of(1.9), RPM.of(1900), Degrees.of(35));
    fill(Meters.of(2), RPM.of(2000), Degrees.of(30));

  }
}
