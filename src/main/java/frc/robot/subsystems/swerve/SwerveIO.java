package frc.robot.subsystems.swerve;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

@Logged
public interface SwerveIO {
	/**
	 * Stops all swerve motors immediately.
	 */
	void stop();

	/**
	 * Get the current heading as provided by the connected gyro.
	 *
	 * @return The current heading
	 */
	Rotation2d getHeading();

  /**
   * Checks whether the gyro is connected to the robot using the built-in Pigeon libraries.
   * @return True if the gyro is connected, false if not.
   */
  boolean isGyroConnected();

  boolean[] getSwerveStatuses();

	/**
	 * Set the gyro to a passed in heading, can be reset to zero by passing in {@link Rotation2d#kZero}
	 *
	 * @param heading
	 *            The heading to reset the gyro to
	 */
	void setGyro(Rotation2d heading);

	/**
	 * Sets a desired state for all the swerve modules to be at in order to drive at the passed in speeds.
	 *
	 * @param speeds
	 *            The speeds to drive at.
	 */
	void driveSpeeds(ChassisSpeeds speeds);

	/**
	 * Get an array of all the module positions for the swerve modules
	 *
	 * @return The modules positions.
	 */
	SwerveModulePosition[] getModulePositions();
}
