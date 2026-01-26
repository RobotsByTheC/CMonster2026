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

  public InterpolatingShooterTreeMap() {
    super(null, null, Comparator.naturalOrder());
  }
}
