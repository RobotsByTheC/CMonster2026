package frc.robot.subsystems.drive;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.LinearAcceleration;
import frc.robot.Constants.DriveConstants;
import frc.robot.logging.Issuable;
import frc.robot.subsystems.drive.swerve.SwerveModule;

/**
 * Common input-output interface for the swerve drive. This is responsible for assigning
 * higher-level goals and reading the current state of the chassis.
 */
@Logged
public interface SwerveIO extends AutoCloseable, Issuable {
  SwerveModule frontLeft();

  SwerveModule frontRight();

  SwerveModule rearLeft();

  SwerveModule rearRight();

  /** Gets the current heading of the chassis. */
  Rotation2d getHeading();

  default boolean isGyroConnected() {
    return false;
  }

  void resetHeading(Rotation2d heading);

  /**
   * Zeroes out the internal heading. Whatever direction the robot is currently facing will be the
   * new zero point from which all later calls to {@link #getHeading()} will be relative to.
   */
  void zeroHeading();

  /**
   * Stops driving and halts movement. The robot may skid or continue to coast for a short period
   * after motor outputs are set to zero.
   */
  @SuppressWarnings("unused")
  default void stop() {
    frontLeft().stop();
    frontRight().stop();
    rearLeft().stop();
    rearRight().stop();
  }

  /**
   * Gets the positions of all four modules. Modules are indexed like so:
   *
   * <ol>
   *   <li>Front left
   *   <li>Front right
   *   <li>Rear left
   *   <li>Rear right
   * </ol>
   */
  default SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      frontLeft().getPosition(),
      frontRight().getPosition(),
      rearLeft().getPosition(),
      rearRight().getPosition()
    };
  }

  /**
   * Gets the states of all four modules. Modules are indexed like so:
   *
   * <ol>
   *   <li>Front left
   *   <li>Front right
   *   <li>Rear left
   *   <li>Rear right
   * </ol>
   */
  default SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      frontLeft().getState(), frontRight().getState(), rearLeft().getState(), rearRight().getState()
    };
  }

  /**
   * Sets the desired states for all four modules. Module states should be indexed like so:
   *
   * <ol>
   *   <li>Front left
   *   <li>Front right
   *   <li>Rear left
   *   <li>Rear right
   * </ol>
   */
  default void setDesiredModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.maxSpeed);
    frontLeft().setDesiredState(desiredStates[0]);
    frontRight().setDesiredState(desiredStates[1]);
    rearLeft().setDesiredState(desiredStates[2]);
    rearRight().setDesiredState(desiredStates[3]);
  }

  /**
   * Resets the internal module distances to 0 meters. Future results from {@link
   * #getModulePositions()} will return distance values relative to their current positions.
   */
  default void resetEncoders() {
    frontLeft().resetEncoders();
    frontRight().resetEncoders();
    rearLeft().resetEncoders();
    rearRight().resetEncoders();
  }

  @Override
  default void close() {
    frontLeft().close();
    frontRight().close();
    rearLeft().close();
    rearRight().close();
  }

  default void setDesiredStateWithoutOptimization(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.maxSpeed);
    frontLeft().setDesiredStateWithoutOptimization(desiredStates[0]);
    frontRight().setDesiredStateWithoutOptimization(desiredStates[1]);
    rearLeft().setDesiredStateWithoutOptimization(desiredStates[2]);
    rearRight().setDesiredStateWithoutOptimization(desiredStates[3]);
  }

  LinearAcceleration getForwardAcceleration();
}
