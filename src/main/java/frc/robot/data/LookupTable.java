package frc.robot.data;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Preferences;
import java.util.EnumSet;
import java.util.Map;

import static edu.wpi.first.units.Units.*;

public class LookupTable {
  private static Angle angle = Radians.zero();
  private static AngularVelocity velocity = RadiansPerSecond.zero();

  private static final class TuningKeys {
    private static String makeKey(int distanceMeters, String type) {
      return "Shooter Tuning - " + distanceMeters + " meters - " + type;
    }

    private static final class RPM {
      public static final String ZERO_METERS = makeKey(0, "RPM");
      public static final String ONE_METERS = makeKey(1, "RPM");
      public static final String TWO_METERS = makeKey(2, "RPM");
      public static final String THREE_METERS = makeKey(3, "RPM");
      public static final String FOUR_METERS = makeKey(4, "RPM");
      public static final String FIVE_METERS = makeKey(5, "RPM");

      public static final Map<Distance, String> ALL_KEYS = Map.of(Meters.of(0), ZERO_METERS, Meters.of(1), ONE_METERS,
          Meters.of(2), TWO_METERS, Meters.of(3), THREE_METERS, Meters.of(4), FOUR_METERS, Meters.of(5), FIVE_METERS);
    }

    private static final class Angle {
      public static final String ZERO_METERS = makeKey(0, "Angle");
      public static final String ONE_METERS = makeKey(1, "Angle");
      public static final String TWO_METERS = makeKey(2, "Angle");
      public static final String THREE_METERS = makeKey(3, "Angle");
      public static final String FOUR_METERS = makeKey(4, "Angle");
      public static final String FIVE_METERS = makeKey(5, "Angle");

      public static final Map<Distance, String> ALL_KEYS = Map.of(Meters.of(0), ZERO_METERS, Meters.of(1), ONE_METERS,
          Meters.of(2), TWO_METERS, Meters.of(3), THREE_METERS, Meters.of(4), FOUR_METERS, Meters.of(5), FIVE_METERS);
    }
  }

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

  private static final EnumSet<NetworkTableEvent.Kind> flags = EnumSet.of(NetworkTableEvent.Kind.kImmediate);

  static {
    // Hardcoded defaults - change these
    fill(Meters.of(0), RPM.of(800), Degrees.of(0));
    fill(Meters.of(1), RPM.of(1200), Degrees.of(2.5));
    fill(Meters.of(2), RPM.of(1500), Degrees.of(5));
    fill(Meters.of(3), RPM.of(1800), Degrees.of(7.5));
    fill(Meters.of(4), RPM.of(2400), Degrees.of(10));
    fill(Meters.of(5), RPM.of(2800), Degrees.of(12.5));

    // Initialize tuning. Preferences are stored in a text file on the roboRIO and read at bootup.
    // These initial values are only written if these preferences don't already exist.
    Preferences.initDouble(TuningKeys.RPM.ZERO_METERS, 800);
    Preferences.initDouble(TuningKeys.Angle.ZERO_METERS, 0);

    Preferences.initDouble(TuningKeys.RPM.ONE_METERS, 1200);
    Preferences.initDouble(TuningKeys.Angle.ONE_METERS, 2.5);

    Preferences.initDouble(TuningKeys.RPM.TWO_METERS, 1500);
    Preferences.initDouble(TuningKeys.Angle.TWO_METERS, 5);

    Preferences.initDouble(TuningKeys.RPM.THREE_METERS, 1800);
    Preferences.initDouble(TuningKeys.Angle.THREE_METERS, 7.5);

    Preferences.initDouble(TuningKeys.RPM.FOUR_METERS, 2400);
    Preferences.initDouble(TuningKeys.Angle.FOUR_METERS, 10);

    Preferences.initDouble(TuningKeys.RPM.FIVE_METERS, 2800);
    Preferences.initDouble(TuningKeys.Angle.FIVE_METERS, 12.5);

    TuningKeys.RPM.ALL_KEYS.forEach((distance, key) -> {
      NetworkTableInstance.getDefault().addListener(new String[]{key}, flags, event -> {
        SPEED_TABLE.put(distance, RPM.of(event.valueData.value.getDouble()));
      });
    });

    TuningKeys.Angle.ALL_KEYS.forEach((distance, key) -> {
      NetworkTableInstance.getDefault().addListener(new String[]{key}, flags, event -> {
        ANGLE_TABLE.put(distance, Degrees.of(event.valueData.value.getDouble()));
      });
    });
  }
}
