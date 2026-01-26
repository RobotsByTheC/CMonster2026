package frc.robot.subsystems.data;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

import java.util.Comparator;

public class InterpolatingShooterTreeMap extends InterpolatingTreeMap<Distance, Pair<AngularVelocity, Angle>> {
  public InterpolatingShooterTreeMap(InverseInterpolator<Distance> inverseInterpolator, Interpolator<Pair<AngularVelocity, Angle>> interpolator, Comparator<Distance> comparator) {
    super(inverseInterpolator, interpolator, comparator);
  }
}
