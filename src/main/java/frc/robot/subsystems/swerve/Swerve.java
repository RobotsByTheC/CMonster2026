package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static frc.robot.Constants.SwerveConstants.DriveConstants;
import static frc.robot.Constants.SwerveConstants.TurnConstants;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;

@Logged
public class Swerve extends SubsystemBase {
	private final SwerveIO io;

	private final PIDController xController;
	private final PIDController yController;
	private final ProfiledPIDController thetaController;
	private final HolonomicDriveController driveController;
	private final SwerveDrivePoseEstimator poseEstimator;

	public Pose2d targetPose = Pose2d.kZero;

	public Swerve(SwerveIO io) {
		this.io = io;

		xController = new PIDController(DriveConstants.DISTANCE_P, DriveConstants.DISTANCE_I, DriveConstants.DISTANCE_D);
		yController = new PIDController(DriveConstants.DISTANCE_P, DriveConstants.DISTANCE_I, DriveConstants.DISTANCE_D);
		thetaController = new ProfiledPIDController(TurnConstants.DISTANCE_P, TurnConstants.DISTANCE_I,
				TurnConstants.DISTANCE_D,
				new TrapezoidProfile.Constraints(TurnConstants.MAX_TURN_SPEED.in(RadiansPerSecond),
						TurnConstants.MAX_TURN_ACCELERATION.in(RadiansPerSecondPerSecond)));
		driveController = new HolonomicDriveController(xController, yController, thetaController);

		poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.KINEMATICS, Rotation2d.kZero,
				io.getModulePositions(), Pose2d.kZero);
	}

	@Override
	public void periodic() {
		poseEstimator.update(io.getHeading(), io.getModulePositions());
	}

	public Command o_zeroGyro() {
		return Commands.runOnce(() -> io.setGyro(Rotation2d.kZero));
	}

	public Command f_drive(Supplier<LinearVelocity> xSpeed, Supplier<LinearVelocity> ySpeed, Supplier<AngularVelocity> rotSpeed) {
		return run(
				() -> io.driveSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed.get(), ySpeed.get(), rotSpeed.get(), io.getHeading())));
	}

	public Command l_driveToPose(Pose2d relativePose) {
		return startRun(
        () -> targetPose = relativePose,
        () -> io.driveSpeeds(driveController.calculate(
            poseEstimator.getEstimatedPosition(), targetPose, 0, targetPose.getRotation()))).until(driveController::atReference);
	}
}
