package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveIO {
  void stop();

  Rotation2d getHeading();

  void setDesiredStates(SwerveModuleState[] states);
}