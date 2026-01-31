package frc.robot.data;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.LinearAcceleration;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

public class ChassisSpeedsFilter {
	private final SlewRateLimiter xFilter;
	private final SlewRateLimiter yFilter;
	private final SlewRateLimiter thetaFilter;

	public ChassisSpeedsFilter(LinearAcceleration linear, AngularAcceleration angular) {
		xFilter = new SlewRateLimiter(linear.in(MetersPerSecondPerSecond));
		yFilter = new SlewRateLimiter(linear.in(MetersPerSecondPerSecond));
		thetaFilter = new SlewRateLimiter(angular.in(RadiansPerSecondPerSecond));
	}

	public void reset() {
		xFilter.reset(0);
		yFilter.reset(0);
		thetaFilter.reset(0);
	}

	public ChassisSpeeds calculate(ChassisSpeeds speeds) {
		speeds.vxMetersPerSecond = xFilter.calculate(speeds.vxMetersPerSecond);
		speeds.vyMetersPerSecond = yFilter.calculate(speeds.vyMetersPerSecond);
		speeds.omegaRadiansPerSecond = thetaFilter.calculate(speeds.omegaRadiansPerSecond);
		return speeds;
	}
}
