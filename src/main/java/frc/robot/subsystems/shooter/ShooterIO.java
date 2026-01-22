package frc.robot.subsystems.shooter;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

@Logged
public interface ShooterIO {
	/**
	 * Stop the flywheel of the shooter.
	 */
	void stop();

	/**
	 * Set the desired velocity of the flywheel, to be reached via PID and FeedForward within the implementations.
	 *
	 * @param angularVelocity
	 *            The speed that the flywheel should go to.
	 */
	void setDesiredVelocity(AngularVelocity angularVelocity);

	/**
	 * Returns the current velocity that the flywheel in the shooter is running at.
	 *
	 * @return The flywheel's velocity.
	 */
	AngularVelocity getVelocity();

	/**
	 * Returns the current draw that the mechanism is currently consuming.
	 *
	 * @return The mechanism's current draw.
	 */
	Current getCurrentDraw();
}
