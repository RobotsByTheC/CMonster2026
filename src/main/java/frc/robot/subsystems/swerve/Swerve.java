package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.Constants.SwerveConstants.DRIVETRAIN_HEIGHT;
import static frc.robot.Constants.SwerveConstants.DRIVETRAIN_WIDTH;
import static frc.robot.Constants.SwerveConstants.MAX_DRIVE_SPEED;
import static frc.robot.Constants.SwerveConstants.MAX_TURN_SPEED;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
  private final SwerveIO io;
  private final SwerveDriveKinematics kinematics;

  public Swerve(SwerveIO io) {
    this.io = io;

    kinematics = new SwerveDriveKinematics(
        new Translation2d(DRIVETRAIN_HEIGHT.in(Meters) / 2, DRIVETRAIN_WIDTH.in(Meters) / 2),
        new Translation2d(DRIVETRAIN_HEIGHT.in(Meters) / 2, -DRIVETRAIN_WIDTH.in(Meters) / 2),
        new Translation2d(-DRIVETRAIN_HEIGHT.in(Meters) / 2, DRIVETRAIN_WIDTH.in(Meters) / 2),
        new Translation2d(-DRIVETRAIN_HEIGHT.in(Meters) / 2, -DRIVETRAIN_WIDTH.in(Meters) / 2)
    );
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    double xSpeedDelivered = xSpeed * MAX_DRIVE_SPEED.in(MetersPerSecond);
    double ySpeedDelivered = ySpeed * MAX_DRIVE_SPEED.in(MetersPerSecond);
    double rotDelivered = rot * MAX_TURN_SPEED.in(RadiansPerSecond);

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
            io.getHeading())
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, MAX_DRIVE_SPEED);
    io.setDesiredStates(swerveModuleStates);
  }
}