package frc.robot.subsystems.data;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

import java.util.Comparator;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

public class InterpolatingShooterTreeMap extends InterpolatingTreeMap<Distance, Pair<AngularVelocity, Angle>> {
  private static final Interpolator<Pair<AngularVelocity, Angle>> INTERPOLATOR = (startValue, endValue, t) -> new Pair<>(
      RadiansPerSecond.of(MathUtil.interpolate(startValue.getFirst().in(RadiansPerSecond), endValue.getFirst().in(RadiansPerSecond), t)),
      Radians.of(MathUtil.interpolate(startValue.getSecond().in(Radians), endValue.getSecond().in(Radians), t)));

  public InterpolatingShooterTreeMap() {
    super(null, INTERPOLATOR, Comparator.naturalOrder());
  }
}
