package frc.robot.subsystems.swerve;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

@Logged
public interface SwerveIO {
	void stop();

	Rotation2d getHeading();

	void setGyro(Rotation2d heading);

	void driveSpeeds(ChassisSpeeds speeds);

	SwerveModulePosition[] getModulePositions();
}
