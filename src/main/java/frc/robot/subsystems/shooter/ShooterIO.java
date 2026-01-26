package frc.robot.subsystems.shooter;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

@Logged
public interface ShooterIO {
	/**
	 * Stop the flywheel of the shooter.
	 */
	void stopFlywheel();

	void stopIntermediary();

	void stopHood();

	/**
	 * Returns the current velocity that the flywheel in the shooter is running at.
	 *
	 * @return The flywheel's velocity.
	 */
	AngularVelocity getFlywheelVelocity();

	Angle getHoodAngle();

	/**
	 * Returns the current draw that the mechanism is currently consuming.
	 *
	 * @return The mechanism's current draw.
	 */
	Current getCurrentDraw();

	/**
	 * Set the desired velocity of the flywheel, to be reached via PID and FeedForward within the implementations.
	 *
	 * @param velocity
	 *            The speed that the flywheel should go to.
	 */
	void setFlywheelVelocity(AngularVelocity velocity);

	void setIntermediaryVoltage(Voltage voltage);

	void setHoodAngle(Angle angle);
}
