package frc.robot.sim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;

/**
 * Simulates a mechanism and determines the current draw pulled by the mechanism over time. All mechanism simulations should be
 * run by the same {@link SimulationContext} in order to accurately determine overall behavior like voltage drop on the battery
 * caused by the current loads of all the powered mechanisms.
 */
public interface MechanismSim {
	/** Gets the current draw of the mechanism, in amps. */
	double getCurrentDraw();

	/**
	 * Updates the simulation by stepping the simulation forward in time.
	 *
	 * @param timestep
	 *            how much time has elapsed since the most recent update, in seconds.
	 */
	void update(double timestep);

	/**
	 * Computes the voltage that can actually be applied. A disabled robot will always output 0 volts due to motor safety. Otherwise,
	 * the input voltage will be bound to be no greater than the current battery voltage.
	 *
	 * @param inputVoltage
	 *            the desired voltage
	 * @return the actual voltage to apply
	 */
	default double outputVoltage(double inputVoltage) {
		if (DriverStation.isDisabled()) {
			return 0;
		}

		return MathUtil.clamp(inputVoltage, -getBatteryVoltage(), getBatteryVoltage());
	}

	/**
	 * Gets the current voltage of the battery, in volts. In simulation, this will be calculated by the
	 * {@link SimulationContext#update(double)} method. This is a convenience method for {@link RobotController#getBatteryVoltage()}.
	 * Note that this will lag behind the simulation by one timestep - for example, a mechanism drawing 100 amps of current will see
	 * the full 12 volts of voltage available on the first call to {@link #update(double)}, but see the voltage drop to 10.5 volts on
	 * the second call due to the induced voltage drop (assuming a battery with 15 mOhm internal resistance).
	 */
	default double getBatteryVoltage() {
		return RobotController.getBatteryVoltage();
	}
}
