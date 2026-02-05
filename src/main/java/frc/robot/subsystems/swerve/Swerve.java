package frc.robot.subsystems.swerve;

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
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PoseEstimation;
import frc.robot.data.ChassisSpeedsFilter;

import java.util.function.Supplier;

@Logged
public class Swerve extends SubsystemBase {
	private final SwerveIO io;

	private final PIDController xController;
	private final PIDController yController;
	private final ProfiledPIDController thetaController;
	private final HolonomicDriveController driveController;
	private final ChassisSpeedsFilter filter;

	private Pose2d targetPose = Pose2d.kZero;

	public Swerve(SwerveIO io) {
		this.io = io;

		xController = new PIDController(DriveConstants.AUTO_P, DriveConstants.AUTO_I, DriveConstants.AUTO_D);
		yController = new PIDController(DriveConstants.AUTO_P, DriveConstants.AUTO_I, DriveConstants.AUTO_D);
		thetaController = new ProfiledPIDController(TurnConstants.AUTO_P, TurnConstants.AUTO_I, TurnConstants.AUTO_D,
				new TrapezoidProfile.Constraints(TurnConstants.MAX_TURN_SPEED.in(RadiansPerSecond),
						TurnConstants.MAX_TURN_ACCELERATION.in(RadiansPerSecondPerSecond)));
		driveController = new HolonomicDriveController(xController, yController, thetaController);
		filter = new ChassisSpeedsFilter(DriveConstants.MAX_DRIVE_ACCELERATION, TurnConstants.MAX_TURN_ACCELERATION);
	}

	@NotLogged
	public Rotation2d getHeading() {
		return io.getHeading();
	}

	@NotLogged
	public SwerveModulePosition[] getModulePositions() {
		return io.getModulePositions();
	}

	public Command o_zeroGyro() {
		return Commands.runOnce(() -> io.setGyro(Rotation2d.kZero));
	}

	public Command f_drive(Supplier<LinearVelocity> vX, Supplier<LinearVelocity> vY, Supplier<AngularVelocity> vTheta) {
		return run(() -> io
				.driveSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(vX.get(), vY.get(), vTheta.get(), io.getHeading())));
	}

	public Command l_driveToPose(Pose2d targetRelativePose, Supplier<Pose2d> currentPose) {
		return startRun(() -> {
			targetPose = targetRelativePose;
			filter.reset(io.getHeading());
		}, () -> io.driveSpeeds(filter.calculate(driveController.calculate(currentPose.get(),
				targetPose, 0, targetPose.getRotation())))).until(driveController::atReference);
	}

	public Command f_driveLocked(Supplier<LinearVelocity> vX, Supplier<LinearVelocity> vY, Supplier<Angle> lockedAngle) {
		return startRun(() -> thetaController.reset(io.getHeading().getRadians()),
				() -> io.driveSpeeds(
						ChassisSpeeds.fromFieldRelativeSpeeds(vX.get(), vY.get(),
								RadiansPerSecond.of(thetaController.calculate(io.getHeading().getRadians(),
										lockedAngle.get().in(Radians))), io.getHeading())
        ));
	}
}
