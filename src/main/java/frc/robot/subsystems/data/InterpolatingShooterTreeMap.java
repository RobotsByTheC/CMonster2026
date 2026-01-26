package frc.robot.subsystems.data;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

import java.util.Comparator;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

public class InterpolatingShooterTreeMap extends InterpolatingTreeMap<Distance, Pair<AngularVelocity, Angle>> {
InverseInterpolator<Pair<AngularVelocity, Angle>> inverseInterpolator = (startValue, endValue, q) -> {
  AngularVelocity range = endValue.getFirst().minus(startValue.getFirst());
  AngularVelocity query = q.getFirst().minus(startValue.getFirst());
  return (range.lte(RadiansPerSecond.zero()) || query.lte(RadiansPerSecond.zero())) ? 0 : query.div(range).baseUnitMagnitude();
};

  public InterpolatingShooterTreeMap() {
    super(null, (startValue, endValue, t) -> new Pair<>(startValue.getFirst().plus(endValue.getFirst().minus(startValue.getFirst())).times(MathUtil.clamp(t, 0, 1)), startValue.getSecond().plus(endValue.getSecond().minus(startValue.getSecond())).times(MathUtil.clamp(t, 0, 1))), Comparator.naturalOrder());
  }
}
