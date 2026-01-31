package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class SimSwerveIO implements SwerveIO {
	@Override
	public void stop() {}

	@Override
	public Rotation2d getHeading() {
		return Rotation2d.kZero;
	}

  @Override
  public boolean isGyroConnected() {
    return true;
  }

  @Override
  public boolean[] getSwerveStatuses() {
    return new boolean[0];
  }

  @Override
	public void setGyro(Rotation2d heading) {}

	@Override
	public void driveSpeeds(ChassisSpeeds speeds) {}

	@Override
	public SwerveModulePosition[] getModulePositions() {
		return new SwerveModulePosition[]{new SwerveModulePosition(), new SwerveModulePosition(),
				new SwerveModulePosition(), new SwerveModulePosition()};
	}
}
