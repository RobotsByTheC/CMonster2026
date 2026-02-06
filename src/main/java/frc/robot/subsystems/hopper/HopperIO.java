package frc.robot.subsystems.hopper;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

@Logged
public interface HopperIO {
	/**
	 * Stop the flywheel of the hopper
	 */
	void stop();

	/**
	 * Returns the current draw that the mechanism is currently consuming.
	 *
	 * @return The mechanism's current draw.
	 */
	Current getCurrentDraw();

	/**
	 * Set the desired voltage of the flywheel.
	 *
	 * @param voltage
	 *            The voltage that the flywheel should be fed.
	 */
	void setVoltage(Voltage voltage);
}
