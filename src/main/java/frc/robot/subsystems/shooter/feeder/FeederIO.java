package frc.robot.subsystems.shooter.feeder;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

@Logged
public interface FeederIO {
	/**
	 * Stop the feeder
	 */
	void stop();

	/**
	 * Returns the current draw that the mechanism is currently consuming.
	 *
	 * @return The mechanism's current draw.
	 */
	Current getCurrentDraw();

	/**
	 * Set the desired voltage of the feeder.
	 *
	 * @param voltage
	 *            The voltage that the feeder should be fed.
	 */
	void setVoltage(Voltage voltage);

	/**
	 * Use distance sensors to determine whether a Fuel has left the feeder and is now inside the flywheel mechanism.
	 *
	 * @return Whether the distance is small enough to confirm that a Fuel has left.
	 */
	boolean isBallAtFlywheel();

	/**
	 * Use distance senors to determine whether a Fuel is currently stored at the base of the feeder.
	 *
	 * @return Whether the distance is small enough to confirm that a Fuel is stored.
	 */
	boolean isBallReadyToFire();
}
