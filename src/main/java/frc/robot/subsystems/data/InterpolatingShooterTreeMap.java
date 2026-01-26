package frc.robot.subsystems.data;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

import java.util.Comparator;

import static edu.wpi.first.units.Units.*;

public class InterpolatingShooterTreeMap extends InterpolatingTreeMap<Distance, Pair<AngularVelocity, Angle>> {
	public InterpolatingShooterTreeMap() {
		super((startValue, endValue, q) -> {
			Distance range = endValue.minus(startValue);
			Distance query = q.minus(startValue);
			return (range.lte(Meters.zero()) || query.lte(Meters.zero())) ? 0 : query.div(range).baseUnitMagnitude();
		}, (startValue, endValue, t) -> new Pair<>(
				startValue.getFirst().plus(endValue.getFirst().minus(startValue.getFirst()))
						.times(MathUtil.clamp(t, 0, 1)),
				startValue.getSecond().plus(endValue.getSecond().minus(startValue.getSecond()))
						.times(MathUtil.clamp(t, 0, 1))),
				Comparator.naturalOrder());
	}
}
