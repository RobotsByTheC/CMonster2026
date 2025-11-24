// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final LinearVelocity maxSpeed = MetersPerSecond.of(4.5);
    public static final AngularVelocity maxAngularSpeed = RotationsPerSecond.of(2.0);
    public static final AngularVelocity slowAngularSpeed = RotationsPerSecond.of(0.5);

    public static final int gyroCanID = 50;

    // Chassis configuration
    public static final Distance trackWidth = Inches.of(24.5);
    // Distance between centers of right and left wheels on robot
    public static final Distance wheelBase = Inches.of(24.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics driveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase.div(2), trackWidth.div(2)),
            new Translation2d(wheelBase.div(2), trackWidth.div(-2)),
            new Translation2d(wheelBase.div(-2), trackWidth.div(2)),
            new Translation2d(wheelBase.div(-2), trackWidth.div(-2)));

    // Angular offsets here describe how the swerve modules are physically rotated with respect
    // to the chassis. There should be offsets at 0, 90, 180, and 270 degrees for a rectangular
    // chassis.
    public static final Rotation2d frontLeftChassisAngularOffset = Rotation2d.fromDegrees(0);
    public static final Rotation2d frontRightChassisAngularOffset = Rotation2d.fromDegrees(0);
    public static final Rotation2d rearLeftChassisAngularOffset = Rotation2d.fromDegrees(0);
    public static final Rotation2d rearRightChassisAngularOffset = Rotation2d.fromDegrees(0);

    // SPARK MAX CAN IDs
    public static final int frontLeftDrivingCanId = 2;
    public static final int rearLeftDrivingCanId = 4;
    public static final int frontRightDrivingCanId = 8;
    public static final int rearRightDrivingCanId = 6;

    public static final int frontLeftTurningCanId = 1;
    public static final int rearLeftTurningCanId = 3;
    public static final int frontRightTurningCanId = 7;
    public static final int rearRightTurningCanId = 5;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int drivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean turningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final Distance wheelDiameter = Inches.of(3 / 1.08);
    public static final Distance wheelCircumference = wheelDiameter.times(Math.PI);
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the
    // bevel pinion
    public static final double drivingMotorReduction = (45.0 * 22) / (drivingMotorPinionTeeth * 15);
    // Gear ratio of the turning (aka "azimuth") motor. ~46.42:1
    public static final double turningMotorReduction = 9424.0 / 203;
    public static final LinearVelocity driveWheelFreeSpeed =
        wheelCircumference
            .times(NeoMotorConstants.freeSpeedRpm.in(RotationsPerSecond))
            .div(drivingMotorReduction)
            .per(Second);

    public static final Distance drivingEncoderPositionFactor =
        wheelDiameter.times(Math.PI / drivingMotorReduction);
    public static final LinearVelocity drivingEncoderVelocityFactor =
        wheelDiameter.times(Math.PI / drivingMotorReduction).per(Minute);

    public static final Angle turningEncoderPositionFactor = Rotations.of(1.0);
    public static final AngularVelocity turningEncoderVelocityFactor = RotationsPerSecond.of(1.0);

    public static final Current drivingCurrentLimit = Amps.of(50);
    public static final Current turningCurrentLimit = Amps.of(20);

    public static final double drivingP = 0.04; //0.04 previous
    public static final double drivingI = 0; //0 previous
    public static final double drivingD = 0; //0 previous
    public static final double drivingFF = 1 / driveWheelFreeSpeed.in(MetersPerSecond);

    public static final double turningP = 1;
    public static final double turningI = 0;
    public static final double turningD = 0;
    public static final double turningFF = 0;

    public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    static {
      // Note: All unit-based configuration values should use SI units (meters, radians, seconds,
      // etc.) for consistency

      drivingConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit((int) drivingCurrentLimit.in(Amps))
          .inverted(true);
      drivingConfig
          .encoder
          .positionConversionFactor(drivingEncoderPositionFactor.in(Meters))
          .velocityConversionFactor(drivingEncoderVelocityFactor.in(MetersPerSecond));
      drivingConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(drivingP, drivingI, drivingD)
          .velocityFF(drivingFF)
          .outputRange(-1, 1);

      turningConfig.idleMode(IdleMode.kBrake).smartCurrentLimit((int) turningCurrentLimit.in(Amps));
      turningConfig
          .absoluteEncoder
          .inverted(turningEncoderInverted)
          .positionConversionFactor(turningEncoderPositionFactor.in(Radians))
          .velocityConversionFactor(turningEncoderVelocityFactor.in(RadiansPerSecond));
      turningConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          .pid(turningP, turningI, turningD)
          .velocityFF(turningFF)
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(0, Rotations.one().in(Radians));
    }
  }

  public static final class VisionConstants {
    public static final Pose3d leftOffset =
        new Pose3d(
            Inches.of(10.5), // X, forward
            Inches.of(12.4), // Y, left
            Inches.of(5.75), // Z, up
            new Rotation3d(
                Degrees.of(0), // Roll, twist
                Degrees.of(30), // Pitch, up
                Degrees.of(15) // Yaw, left
                ));
    public static final Pose3d rightOffset =
        new Pose3d(
            Inches.of(10.5), // X, forward
            Inches.of(-12.4), // Y, left
            Inches.of(5.75), // Z, up
            new Rotation3d(
                Degrees.of(0), // Roll, twist
                Degrees.of(30), // Pitch, up
                Degrees.of(-15) // Yaw, left
                ));
  }

  public static final class OIConstants {
    public static final int driverControllerPort = 1;
    public static final int leftJoystickPort = 2;
    public static final int rightJoystickPort = 3;
  }

  public static final class AutoConstants {
    public static final double pXController = 1.5;
    public static final double pYController = 1.5;
    public static final double pThetaController = 4;
  }

  public static final class NeoMotorConstants {
    public static final AngularVelocity freeSpeedRpm = RPM.of(5676);
  }
}
