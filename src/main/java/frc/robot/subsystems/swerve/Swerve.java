package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static frc.robot.Constants.SwerveConstants.DriveConstants;
import static frc.robot.Constants.SwerveConstants.TurnConstants;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.data.ChassisSpeedsFilter;
import java.util.function.Supplier;

@Logged
public class Swerve extends SubsystemBase {
  public enum DriveState {
    NORMAL,
    LOCKED
  }

  private final SwerveIO io;

  private final ProfiledPIDController thetaController;
  private final HolonomicDriveController driveController;
  private final ChassisSpeedsFilter filter;

  public Swerve(SwerveIO io) {
    this.io = io;

    thetaController = new ProfiledPIDController(TurnConstants.AUTO_P, TurnConstants.AUTO_I, TurnConstants.AUTO_D,
        new TrapezoidProfile.Constraints(TurnConstants.MAX_TURN_SPEED.in(RadiansPerSecond),
            TurnConstants.MAX_TURN_ACCELERATION.in(RadiansPerSecondPerSecond)));
    thetaController.enableContinuousInput(-Math.PI/2, Math.PI/2);
    driveController = new HolonomicDriveController(new PIDController(DriveConstants.AUTO_P, DriveConstants.AUTO_I, DriveConstants.AUTO_D), new PIDController(DriveConstants.AUTO_P, DriveConstants.AUTO_I, DriveConstants.AUTO_D), thetaController);
    filter = new ChassisSpeedsFilter(DriveConstants.MAX_DRIVE_ACCELERATION, TurnConstants.MAX_TURN_ACCELERATION);
  }

  @NotLogged
  public Rotation2d getHeading() {
    return io.getHeading();
  }

  @Override
  public void periodic() {
//    if (io.getHeading().getDegrees() > 180){
//      io.setGyro(io.getHeading().minus(new Rotation2d(Math.PI*2)));
//    }
//    if (io.getHeading().getDegrees() < -180) {
//      io.setGyro(io.getHeading().plus(new Rotation2d(Math.PI*2)));
//    }
  }

  @NotLogged
  public SwerveModulePosition[] getModulePositions() {
    return io.getModulePositions();
  }

  public Command f_drive(Supplier<LinearVelocity> vX, Supplier<LinearVelocity> vY, Supplier<AngularVelocity> vTheta) {
    return run(
        () -> io.driveSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(vX.get(), vY.get(), vTheta.get(), io.getHeading())));
  }

  public Command l_driveToPose(Supplier<Pose2d> targetRelativePose, Supplier<Pose2d> currentPose) {
    return startRun(() -> filter.reset(io.getHeading()), () -> io.driveSpeeds(
        filter.calculate(driveController.calculate(currentPose.get(), targetRelativePose.get(), 0, targetRelativePose.get().getRotation()))))
        .until(driveController::atReference);
  }

  public Command o_resetGyro() {
    return Commands.runOnce(() -> {
      DriverStation.getAlliance().ifPresent(a -> {
        switch (a) {
          case Red -> {
            io.setGyro(Rotation2d.k180deg);
            System.out.println("GYRO RESET: 180 degrees (towards blue wall)");
          }
          case Blue -> {
            io.setGyro(Rotation2d.kZero);
            System.out.println("GYRO RESET: 0 degrees (towards red wall)");
          }
        }
      });
    }).ignoringDisable(true);
  }

  public Command o_setGyroToVisionIfPossible(Supplier<Angle> visionEstimate, Supplier<Time> timestamp) {
    Time acceptanceThreshold = Milliseconds.of(100);
    return Commands.runOnce(() -> {
      Time ts = timestamp.get();
      Time dt = RobotController.getMeasureTime().minus(ts);
      if (dt.lte(acceptanceThreshold)) {
        Angle angle = visionEstimate.get();
        double angleRads = angle.in(Radians);
        // ensure we're within ±2π°e
        double twoPi = 2 * Math.PI;
        while (angleRads > twoPi) {
          angleRads -= twoPi;
        }

        while (angleRads < -2 * Math.PI) {
          angleRads += twoPi;
        }

        io.setGyro(new Rotation2d(angleRads));
        System.out.printf("GYRO RESET: Estimated at %.1f deg, computed to %.2f rads.%n", angle.in(Degrees), angleRads);
      } else {
        System.out.println("GYRO RESET FAILED: CAN'T SEE APRILTAG - Timestamp " + ts + " is " + dt + " seconds out of date!");
      }
    }).ignoringDisable(true);
  }

  private void setX() {
    io.setModulePositions(
        new SwerveModulePosition[] {
            new SwerveModulePosition(0, Rotation2d.fromDegrees(45)),
            new SwerveModulePosition(0, Rotation2d.fromDegrees(-45)),
            new SwerveModulePosition(0, Rotation2d.fromDegrees(-45)),
            new SwerveModulePosition(0, Rotation2d.fromDegrees(45))
        });
  }

  public Command commandSetX() {
    return run(this::setX);
  }

  public AngularVelocity calculateThetaController(Angle measurement, Angle goal) {
    return RadiansPerSecond.of(thetaController.calculate(measurement.in(Radians), goal.in(Radians)));
  }
}
