package frc.robot.subsystems.swerve;

import static frc.robot.Constants.SwerveConstants.DriveConstants;
import static frc.robot.Constants.SwerveConstants.DriveConstants.KINEMATICS;
import static frc.robot.Constants.SwerveConstants.TurnConstants;
import static frc.robot.Constants.SwerveConstants.GYRO_CAN_ID;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

@Logged
public class RealSwerveIO implements SwerveIO {
	private final SwerveModule frontLeft;
	private final SwerveModule frontRight;
	private final SwerveModule backLeft;
	private final SwerveModule backRight;

	private final Pigeon2 gyro;

	public RealSwerveIO() {
		frontLeft = new SwerveModule(DriveConstants.FRONT_LEFT_CAN_ID, TurnConstants.FRONT_LEFT_CAN_ID, -Math.PI / 2);
		frontRight = new SwerveModule(DriveConstants.FRONT_RIGHT_CAN_ID, TurnConstants.FRONT_RIGHT_CAN_ID, 0);
		backLeft = new SwerveModule(DriveConstants.BACK_LEFT_CAN_ID, TurnConstants.BACK_LEFT_CAN_ID, Math.PI);
		backRight = new SwerveModule(DriveConstants.BACK_RIGHT_CAN_ID, TurnConstants.BACK_RIGHT_CAN_ID, Math.PI / 2);

		gyro = new Pigeon2(GYRO_CAN_ID);
	}

	public void setDesiredStates(SwerveModuleState[] states) {
		frontLeft.setDesiredState(states[0]);
		frontRight.setDesiredState(states[1]);
		backLeft.setDesiredState(states[2]);
		backRight.setDesiredState(states[3]);
	}

	@Override
	public void stop() {
		frontLeft.stop();
		frontRight.stop();
		backLeft.stop();
		backRight.stop();
	}

	@Override
	public Rotation2d getHeading() {
		return gyro.getRotation2d();
	}

	@Override
	public void setGyro(Rotation2d heading) {
		gyro.setYaw(heading.getDegrees());
	}

	@Override
	public void driveSpeeds(ChassisSpeeds speeds) {
		setDesiredStates(KINEMATICS.toSwerveModuleStates(speeds));
	}

	@Override
	public SwerveModulePosition[] getModulePositions() {
		return new SwerveModulePosition[]{frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(),
				backRight.getPosition()};
	}
}
