package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.*;

public class Constants {
	public static class SwerveConstants {
		public static class DriveConstants {
			public static final LinearVelocity MAX_DRIVE_SPEED = MetersPerSecond.of(2);
			public static final LinearAcceleration MAX_DRIVE_ACCELERATION = MetersPerSecondPerSecond.of(2);

			public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
					new Translation2d(DRIVETRAIN_HEIGHT.in(Meters) / 2, DRIVETRAIN_WIDTH.in(Meters) / 2),
					new Translation2d(DRIVETRAIN_HEIGHT.in(Meters) / 2, -DRIVETRAIN_WIDTH.in(Meters) / 2),
					new Translation2d(-DRIVETRAIN_HEIGHT.in(Meters) / 2, DRIVETRAIN_WIDTH.in(Meters) / 2),
					new Translation2d(-DRIVETRAIN_HEIGHT.in(Meters) / 2, -DRIVETRAIN_WIDTH.in(Meters) / 2));

			public static final int KP = 1;
			public static final int KI = 0;
			public static final int KD = 0;
			public static final int AUTO_P = 1;
			public static final int AUTO_I = 0;
			public static final int AUTO_D = 0;
		}

		public static class TurnConstants {
			public static final AngularVelocity MAX_TURN_SPEED = RadiansPerSecond.of(2);
			public static final AngularAcceleration MAX_TURN_ACCELERATION = RadiansPerSecondPerSecond.of(2);

			public static final int KP = 1;
			public static final int KI = 0;
			public static final int KD = 0;
			public static final int AUTO_P = 1;
			public static final int AUTO_I = 0;
			public static final int AUTO_D = 0;
		}

		public static final Distance DRIVETRAIN_WIDTH = Inches.of(27);
		public static final Distance DRIVETRAIN_HEIGHT = Inches.of(27);
		public static final Angle TOLERANCE = Degrees.of(2);
	}

	public static class InputConstants {
		public static final int CONTROLLER_PORT = 1;
		public static final int LEFT_JOYSTICK_PORT = 2;
		public static final int RIGHT_JOYSTICK_PORT = 3;
	}

	public static class CANConstants {
		public static final int FRONT_LEFT_DRIVE_CAN_ID = 1;
		public static final int FRONT_LEFT_TURN_CAN_ID = 2;
		public static final int FRONT_RIGHT_DRIVE_CAN_ID = 3;
		public static final int FRONT_RIGHT_TURN_CAN_ID = 4;
		public static final int BACK_LEFT_DRIVE_CAN_ID = 5;
		public static final int BACK_LEFT_TURN_CAN_ID = 6;
		public static final int BACK_RIGHT_DRIVE_CAN_ID = 7;
		public static final int BACK_RIGHT_TURN_CAN_ID = 8;
		public static final int INTAKE_MOTOR_CAN_ID = 9;
		public static final int INTAKE_WRIST_CAN_ID = 10;
		public static final int FLYWHEEL_LEFT_A_CAN_ID = 11;
		public static final int FLYWHEEL_LEFT_B_CAN_ID = 12;
		public static final int FLYWHEEL_RIGHT_A_CAN_ID = 13;
		public static final int FLYWHEEL_RIGHT_B_CAN_ID = 14;
		public static final int INTERMEDIARY_CAN_ID = 15;
		public static final int HOOD_CAN_ID = 16;

		public static final int GYRO_CAN_ID = 50;
	}

	public static class IntakeConstants {
		public static final double KP = 5;
		public static final double KI = 0;
		public static final double KD = 0.85;
		public static final double KS = 0.014847;
		public static final double KG = 0.3732;
		public static final double KV = 1.1878;
		public static final double KA = 0.0024773;

		public static final Voltage INTAKE_VOLTAGE = Volts.of(5);
		public static final Voltage OUTTAKE_VOLTAGE = Volts.of(-5);
		public static final AngularVelocity MAX_WRIST_SPEED = RadiansPerSecond.of(10);
		public static final AngularAcceleration MAX_WRIST_ACCELERATION = RadiansPerSecondPerSecond.of(3);
		public static final Angle WRIST_STOW_ANGLE = Radians.of(Math.PI);
		public static final Angle WRIST_EXTEND_ANGLE = Degrees.of(30);
	}

	public static class ShooterConstants {
		public static class Hood {
			public static final double KP = 1;
			public static final double KI = 0;
			public static final double KD = 0;
			public static final double KS = 0;
			public static final double KV = 0;
		}

		public static class FlywheelConstants {
			public static final double KP = 0.006;
			public static final double KI = 0;
			public static final double KD = 0;
			public static final double KS = 0;
			public static final double KV = 0.00137;
		}
		public static final AngularVelocity SHOOTER_SPEED = RPM.of(5000);
	}
}
