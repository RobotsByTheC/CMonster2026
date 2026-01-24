package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.SwerveConstants.DriveConstants;
import static frc.robot.Constants.SwerveConstants.TOLERANCE;
import static frc.robot.Constants.SwerveConstants.TurnConstants;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

@Logged
public class SwerveModule {
	private final SparkMax driveSpark;
	private final SparkMax turnSpark;
	private final RelativeEncoder driveEncoder;
	private final AbsoluteEncoder turnEncoder;

	private final SparkClosedLoopController driveController;
	private final SparkClosedLoopController turnController;

	private final double offset;
	private SwerveModuleState desiredState = new SwerveModuleState(0, Rotation2d.kZero);

	public SwerveModule(int drivingCan, int turningCan, double offset) {
		driveSpark = new SparkMax(drivingCan, SparkLowLevel.MotorType.kBrushless);
		turnSpark = new SparkMax(turningCan, SparkLowLevel.MotorType.kBrushless);

		driveEncoder = driveSpark.getEncoder();
		turnEncoder = turnSpark.getAbsoluteEncoder();

		driveController = driveSpark.getClosedLoopController();
		turnController = turnSpark.getClosedLoopController();

		SparkMaxConfig driveConfig = new SparkMaxConfig();
		driveConfig.closedLoop.pid(DriveConstants.KP, DriveConstants.KI, DriveConstants.KD)
				.allowedClosedLoopError(TOLERANCE.in(Rotations), ClosedLoopSlot.kSlot0);
		SparkMaxConfig turnConfig = new SparkMaxConfig();
		turnConfig.closedLoop.pid(TurnConstants.kP, TurnConstants.kI, TurnConstants.kD)
				.allowedClosedLoopError(TOLERANCE.in(Rotations), ClosedLoopSlot.kSlot0);

		driveSpark.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		turnSpark.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		this.offset = offset;
		desiredState.angle = new Rotation2d(turnEncoder.getPosition());
		driveEncoder.setPosition(0);
	}

	public SwerveModuleState getState() {
		return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(turnEncoder.getPosition() - offset));
	}

	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(driveEncoder.getPosition(), new Rotation2d(turnEncoder.getPosition() - offset));
	}

	public void setDesiredState(SwerveModuleState desiredState) {
		SwerveModuleState correctedDesiredState = new SwerveModuleState();
		correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
		correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(offset));

		correctedDesiredState.optimize(new Rotation2d(turnEncoder.getPosition()));

		driveController.setSetpoint(correctedDesiredState.speedMetersPerSecond, SparkBase.ControlType.kVelocity);
		turnController.setSetpoint(correctedDesiredState.angle.getRadians(), SparkBase.ControlType.kPosition);

		this.desiredState = desiredState;
	}

	public void resetEncoders() {
		driveEncoder.setPosition(0);
	}

	public void stop() {
		driveSpark.setVoltage(0);
		turnSpark.setVoltage(0);
		driveController.setSetpoint(0, SparkBase.ControlType.kVoltage);
		turnController.setSetpoint(0, SparkBase.ControlType.kVoltage);
	}
}
