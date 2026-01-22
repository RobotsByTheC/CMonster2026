package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.Constants.SwerveConstants.BACK_LEFT_DRIVE_CAN_ID;
import static frc.robot.Constants.SwerveConstants.BACK_LEFT_TURN_CAN_ID;
import static frc.robot.Constants.SwerveConstants.BACK_RIGHT_DRIVE_CAN_ID;
import static frc.robot.Constants.SwerveConstants.BACK_RIGHT_TURN_CAN_ID;
import static frc.robot.Constants.SwerveConstants.FRONT_LEFT_DRIVE_CAN_ID;
import static frc.robot.Constants.SwerveConstants.FRONT_LEFT_TURN_CAN_ID;
import static frc.robot.Constants.SwerveConstants.FRONT_RIGHT_DRIVE_CAN_ID;
import static frc.robot.Constants.SwerveConstants.FRONT_RIGHT_TURN_CAN_ID;
import static frc.robot.Constants.SwerveConstants.GYRO_CAN_ID;
import static frc.robot.Constants.SwerveConstants.MAX_DRIVE_SPEED;
import static frc.robot.Constants.SwerveConstants.MAX_TURN_SPEED;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class RealSwerveIO implements SwerveIO {
  private final SwerveModule frontLeft;
  private final SwerveModule frontRight;
  private final SwerveModule backLeft;
  private final SwerveModule backRight;

  private final Pigeon2 gyro;

  public RealSwerveIO() {
    frontLeft = new SwerveModule(FRONT_LEFT_DRIVE_CAN_ID, FRONT_LEFT_TURN_CAN_ID, -Math.PI);
    frontRight = new SwerveModule(FRONT_RIGHT_DRIVE_CAN_ID, FRONT_RIGHT_TURN_CAN_ID, 0);
    backLeft = new SwerveModule(BACK_LEFT_DRIVE_CAN_ID, BACK_LEFT_TURN_CAN_ID, Math.PI);
    backRight = new SwerveModule(BACK_RIGHT_DRIVE_CAN_ID, BACK_RIGHT_TURN_CAN_ID, Math.PI / 2);

    gyro = new Pigeon2(GYRO_CAN_ID);
  }

  @Override
  public void setDesiredStates() {

  }

  @Override
  public void stop() {

  }
}