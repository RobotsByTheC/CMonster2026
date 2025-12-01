// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.DriveConstants.driveKinematics;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import java.util.Arrays;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

@Logged
public class DriveSubsystem extends SubsystemBase implements AutoCloseable {
  private final Timer timer = new Timer();

  private final Voltage appliedSysidVoltage = Volts.zero();
  private final SwerveIO io;

  private final PIDController xController =
      new PIDController(Constants.AutoConstants.pXController, 0.05, 0.2);

  private final PIDController yController =
      new PIDController(Constants.AutoConstants.pYController, 0.05, 0.2);

  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          Constants.AutoConstants.pThetaController,
          0,
          0,
          new TrapezoidProfile.Constraints(
              DriveConstants.maxAngularSpeed.in(RadiansPerSecond),
              RadiansPerSecondPerSecond.convertFrom(10, RotationsPerSecondPerSecond)));

  private final HolonomicDriveController driveController =
      new HolonomicDriveController(xController, yController, thetaController);

  @Logged public double goal;
  // Odometry class for tracking robot pose
  @NotLogged private final SwerveDrivePoseEstimator visionPoseEstimator;
  @NotLogged private final SwerveDrivePoseEstimator poseEstimator;

  private final Field2d field = new Field2d();

  private BooleanSupplier blegg = driveController::atReference;
  @Logged private DoubleSupplier estimatedX;

  public enum ReferenceFrame {
    ROBOT,
    FIELD
  }

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(SwerveIO io) {
    this.io = io;

    xController.setTolerance(Meters.convertFrom(2, Inches));
    yController.setTolerance(Meters.convertFrom(2, Inches));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    driveController.setTolerance(new Pose2d(Inches.of(2), Inches.of(2), new Rotation2d(Degrees.of(5))));
    poseEstimator =
        new SwerveDrivePoseEstimator(
            driveKinematics,
            Rotation2d.fromDegrees(0),
            io.getModulePositions(),
            new Pose2d(Feet.zero(), Feet.zero(), new Rotation2d(Degrees.zero())),
            // Use default standard deviations of ±4" and ±6° for odometry-derived position data
            // (i.e. 86% of results will be within 4" and 6° of the true value, and 95% will
            // be within ±8" and ±12°)
            VecBuilder.fill(
                Inches.of(4).in(Meters), Inches.of(4).in(Meters), Degrees.of(6).in(Radians)),
            // Use default standard deviations of ±35" and ±52° for vision-derived position data
            VecBuilder.fill(
                Inches.of(35).in(Meters), Inches.of(35).in(Meters), Degrees.of(52).in(Radians)));
    visionPoseEstimator =
        new SwerveDrivePoseEstimator(
            driveKinematics,
            Rotation2d.fromDegrees(0),
            io.getModulePositions(),
            new Pose2d(Feet.zero(), Feet.zero(), new Rotation2d(Degrees.zero())),
            // Use default standard deviations of ±4" and ±6° for odometry-derived position data
            // (i.e. 86% of results will be within 4" and 6° of the true value, and 95% will
            // be within ±8" and ±12°)
            VecBuilder.fill(
                Inches.of(4).in(Meters), Inches.of(4).in(Meters), Degrees.of(6).in(Radians)),
            // Use default standard deviations of ±35" and ±52° for vision-derived position data
            VecBuilder.fill(
                Inches.of(35).in(Meters), Inches.of(35).in(Meters), Degrees.of(52).in(Radians)));
    estimatedX = () -> visionPoseEstimator.getEstimatedPosition().getMeasureX().in(Meters);

    Shuffleboard.getTab("Drive").add("Field", field);
    Shuffleboard.getTab("Drive").add("X PID", xController);
    Shuffleboard.getTab("Drive").add("Y PID", yController);
    Shuffleboard.getTab("Drive").add("Theta PID", thetaController);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    poseEstimator.update(io.getHeading(), io.getModulePositions());
    visionPoseEstimator.update(io.getHeading(), io.getModulePositions());
    field.setRobotPose(poseEstimator.getEstimatedPosition());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  @SuppressWarnings("unused")
  public void resetOdometry(Pose2d pose) {
    io.resetHeading(pose.getRotation());
    poseEstimator.resetPosition(io.getHeading(), io.getModulePositions(), pose);
  }

  public Command zeroGyro() {
    return Commands.runOnce(() -> io.resetHeading(Rotation2d.kZero))
        .ignoringDisable(true)
        .withName("Reset Gyro");
  }

  public void resetHeading() {
    io.resetHeading(Rotation2d.kZero);
  }

  public Command driveDistance(Distance distance) {
    Pose2d[] pose = new Pose2d[1];
    return startRun(
            () ->
                pose[0] =
                    getPose()
                        .transformBy(new Transform2d(distance, Meters.zero(), Rotation2d.kZero)),
            () -> drive(driveController.calculate(getPose(), pose[0], 0, pose[0].getRotation())))
        .withName("Driving Distance " + distance);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward)
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param orientation What reference frame the speeds are in
   */
  public void drive(
      LinearVelocity xSpeed,
      LinearVelocity ySpeed,
      AngularVelocity rot,
      ReferenceFrame orientation) {
    var speeds =
        switch (orientation) {
          case ROBOT -> new ChassisSpeeds(xSpeed, ySpeed, rot);
          case FIELD -> ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, io.getHeading());
        };
    drive(speeds);
  }

  /**
   * Drives the robot with the given chassis speeds. Note that chassis speeds are relative to the
   * robot's reference frame.
   *
   * @param speeds the speeds at which the robot should move
   */
  public void drive(ChassisSpeeds speeds) {
    var swerveModuleStates = driveKinematics.toSwerveModuleStates(speeds);

    io.setDesiredModuleStates(swerveModuleStates);
  }

  public Command myThirdAttemptAtDriveToRobotRelativePose(Supplier<Pose2d> target) {
    return Commands.runOnce(() -> {
      visionPoseEstimator.resetPose(Pose2d.kZero);
      System.out.println(target.get());
        })
        .andThen(
            run( () -> {
                drive(
                    driveController.calculate(
                        visionPoseEstimator.getEstimatedPosition(),
                        target.get().plus(new Transform2d(Meters.of(-0.5), Meters.of(0.1), Rotation2d.kZero)),
                        0,
                        Rotation2d.kZero));
            }).until(driveController::atReference)
        );
  }

  public Command mySecondAttemptAtDriveToRobotRelativePose(
      Supplier<Pose2d> target, Trigger canSeeTarget) {
    return Commands.runOnce(() -> poseEstimator.resetPose(Pose2d.kZero))
        .andThen(
            run(() -> {
                  drive(
                      driveController
                          .calculate(Pose2d.kZero, target.get(), 0, Rotation2d.kZero)
                          .div(3));
                })
                .until(() -> (driveController.atReference())));
  }

  public Command myDriveToPose(Supplier<Pose2d> target) {
    return Commands.runOnce(
            () -> {
              poseEstimator.resetPose(Pose2d.kZero);
            })
        .andThen(
            run(
                () -> {
                  drive(
                      driveController.calculate(
                          poseEstimator.getEstimatedPosition(), target.get(), 1, Rotation2d.kZero));
                  //          System.out.println("Trying to drive to " + target + ", currently at "
                  // + poseEstimator.getEstimatedPosition());
                }));
  }

  public Command driveToRobotRelativePose(Pose2d pose) {
    Pose2d[] startingPose = new Pose2d[1];
    return runOnce(() -> startingPose[0] = getPose())
        //        .andThen(rotateToHeading(pose.getRotation()))
        .andThen(
            run(
                () -> {
                  Pose2d currentPose =
                      new Pose2d(
                          getPose().minus(startingPose[0]).getX(),
                          getPose().minus(startingPose[0]).getY(),
                          getPose().minus(startingPose[0]).getRotation());
                  drive(driveController.calculate(currentPose, pose, 0, pose.getRotation()));
                }))
        .withName("Move to " + pose);
  }

  /** Sets the wheels into an X formation to prevent movement. */
  public void setX() {
    io.setDesiredModuleStates(
        new SwerveModuleState[] {
          new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(45))
        });
  }

  /** Resets the drive encoders to currently read a position of 0. */
  @SuppressWarnings("unused")
  public void resetEncoders() {
    io.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  @SuppressWarnings("unused")
  public void zeroHeading() {
    io.zeroHeading();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public Rotation2d getHeading() {
    return io.getHeading();
  }

  public SwerveModulePosition[] getModulePositions() {
    return io.getModulePositions();
  }

  public SwerveModuleState[] getModuleStates() {
    return io.getModuleStates();
  }

  @SuppressWarnings("unused")
  public Command setXCommand() {
    return run(this::setX)
        .until(
            () ->
                Arrays.stream(getModuleStates())
                    .allMatch(
                        state -> {
                          double v = Math.abs(state.speedMetersPerSecond);
                          double angle = state.angle.getDegrees();
                          return v <= 0.01 && Math.abs(angle % 45) <= 1.5;
                        }))
        .withName("Set X");
  }

  /**
   * Creates a command that uses joystick inputs to drive the robot. The speeds are field-relative.
   *
   * @param x a supplier for the relative speed that the robot should move on the field's X-axis, as
   *     a number from -1 (towards the alliance wall) to +1 (towards the opposing alliance wall).
   * @param y a supplier for the relative speed that the robot should move on the field's Y-axis, as
   *     a number from -1 (towards the right side of the field) to +1 (towards the left side of the
   *     field).
   * @param omega a supplier for the relative speed that the robot should spin about its own axis,
   *     as a number from -1 (maximum clockwise speed) to +1 (maximum counter-clockwise speed).
   * @return the driving command
   */
  public Command driveXYTheta(DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega) {
    var xSpeed = MetersPerSecond.mutable(0);
    var ySpeed = MetersPerSecond.mutable(0);
    var omegaSpeed = RadiansPerSecond.mutable(0);

    return run(() -> {
          xSpeed.mut_setMagnitude(
              MathUtil.applyDeadband(-x.getAsDouble(), 0.01)
                  * DriveConstants.maxSpeed.in(MetersPerSecond));
          ySpeed.mut_setMagnitude(
              MathUtil.applyDeadband(-y.getAsDouble(), 0.01)
                  * DriveConstants.maxSpeed.in(MetersPerSecond));
          omegaSpeed.mut_setMagnitude(
              MathUtil.applyDeadband(-omega.getAsDouble(), 0.15)
                  * DriveConstants.maxAngularSpeed.in(RadiansPerSecond));

          drive(xSpeed, ySpeed, omegaSpeed, ReferenceFrame.FIELD);
        })
        .finallyDo(this::setX)
        .withName("Drive With Joysticks");
  }

  @Override
  public void close() {
    io.close();
  }

  // Creates a SysIdRoutine
  @NotLogged
  private final SysIdRoutine routine =
      new SysIdRoutine(
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(this::voltageDrive, this::logMotors, this));

  private void logMotors(SysIdRoutineLog s) {
    s.motor("frontLeft")
        .linearPosition(Meters.of(io.frontLeft().getPosition().distanceMeters))
        .linearVelocity(MetersPerSecond.of(io.frontLeft().getState().speedMetersPerSecond))
        .voltage(appliedSysidVoltage);
    s.motor("rearLeft")
        .linearPosition(Meters.of(io.rearLeft().getPosition().distanceMeters))
        .linearVelocity(MetersPerSecond.of(io.rearLeft().getState().speedMetersPerSecond))
        .voltage(appliedSysidVoltage);
    s.motor("frontRight")
        .linearPosition(Meters.of(io.frontRight().getPosition().distanceMeters))
        .linearVelocity(MetersPerSecond.of(io.frontRight().getState().speedMetersPerSecond))
        .voltage(appliedSysidVoltage);
    s.motor("rearRight")
        .linearPosition(Meters.of(io.rearRight().getPosition().distanceMeters))
        .linearVelocity(MetersPerSecond.of(io.rearRight().getState().speedMetersPerSecond))
        .voltage(appliedSysidVoltage);
  }

  private void voltageDrive(Voltage v) {
    io.frontLeft().setVoltageForDrivingMotor(v);
    io.frontRight().setVoltageForDrivingMotor(v);
    io.rearLeft().setVoltageForDrivingMotor(v);
    io.rearRight().setVoltageForDrivingMotor(v);
  }

  @SuppressWarnings("unused")
  public Command sysIdQuasiStatic(
      SysIdRoutine.Direction direction) { // can bind to controller buttons
    return routine.quasistatic(direction);
  }

  @SuppressWarnings("unused")
  public Command sysIdDynamic(SysIdRoutine.Direction direction) { // can bind to controller buttons
    return routine.dynamic(direction);
  }

  public Command autoLeaveArea() {
    return Commands.waitTime(Seconds.of(10))
        .andThen(
            run(
                () ->
                    drive(
                        FeetPerSecond.of(-3),
                        FeetPerSecond.zero(),
                        RadiansPerSecond.zero(),
                        ReferenceFrame.FIELD)))
        .withName("Auto Leave Area");
  }

  @SuppressWarnings("unused")
  public Command pointForward() {
    return run(this::setForward)
        .until(
            () ->
                Arrays.stream(getModuleStates())
                    .allMatch(
                        state -> {
                          double angle = state.angle.getDegrees();
                          return 1.5 >= angle && angle >= -1.5;
                        }))
        .withName("Set 0");
  }

  public Command rotateToHeading(Rotation2d heading) {
    // Use a PID controller to control the heading of the robot
    return runOnce(() -> {
        System.out.println("Rotating to heading " + heading);
    }).andThen(() -> {
          AngularVelocity velocity =
              RadiansPerSecond.of(
                  thetaController.calculate(io.getHeading().getRadians(), heading.getRadians()));
          drive(MetersPerSecond.zero(), MetersPerSecond.zero(), velocity, ReferenceFrame.ROBOT);
        })
        .until(thetaController::atSetpoint);
  }

  private void setForward() {
    io.setDesiredStateWithoutOptimization(
        new SwerveModuleState[] {
          new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(0))
        });
  }

  private ChassisSpeeds getSwerveChassisSpeeds() {
    return driveKinematics.toChassisSpeeds(
        io.frontLeft().getState(),
        io.frontRight().getState(),
        io.rearLeft().getState(),
        io.rearLeft().getState());
  }

  public LinearVelocity getAverageVelocity() {
    return MetersPerSecond.of(
        Math.sqrt(
            Math.pow(getSwerveChassisSpeeds().vxMetersPerSecond, 2)
                + Math.pow(getSwerveChassisSpeeds().vyMetersPerSecond, 2)));
  }
}
