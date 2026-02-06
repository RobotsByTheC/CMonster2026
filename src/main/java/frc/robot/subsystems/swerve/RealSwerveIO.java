package frc.robot.subsystems.swerve;

import static frc.robot.Constants.SwerveConstants.DriveConstants.KINEMATICS;
import static frc.robot.Constants.CANConstants.*;

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
    frontLeft = new SwerveModule(FRONT_LEFT_DRIVE_CAN_ID, FRONT_LEFT_TURN_CAN_ID);
    frontRight = new SwerveModule(FRONT_RIGHT_DRIVE_CAN_ID, FRONT_RIGHT_TURN_CAN_ID);
    backLeft = new SwerveModule(BACK_LEFT_DRIVE_CAN_ID, BACK_LEFT_TURN_CAN_ID);
    backRight = new SwerveModule(BACK_RIGHT_DRIVE_CAN_ID, BACK_RIGHT_TURN_CAN_ID);

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
