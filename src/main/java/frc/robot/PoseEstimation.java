package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.VisionConstants.LEFT_CAMERA_OFFSET;
import static frc.robot.Constants.VisionConstants.RIGHT_CAMERA_OFFSET;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.data.PolarPoint;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

@Logged
public class PoseEstimation {
	private final PhotonCamera rightCamera;
	private final PhotonCamera leftCamera;

	private final PhotonPoseEstimator leftEstimator;
	private final PhotonPoseEstimator rightEstimator;
	private final SwerveDrivePoseEstimator swerveEstimator;

	public PoseEstimation() {
		leftCamera = new PhotonCamera("OV9281-2");
		rightCamera = new PhotonCamera("OV9281-1");

		leftEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded),
				LEFT_CAMERA_OFFSET);
		rightEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded),
				RIGHT_CAMERA_OFFSET);
		swerveEstimator = new SwerveDrivePoseEstimator(Constants.SwerveConstants.DriveConstants.KINEMATICS,
				Rotation2d.kZero, new SwerveModulePosition[4], Pose2d.kZero);
	}

	public void update(Rotation2d gyro, SwerveModulePosition[] swervePositions) {
		var leftResults = leftCamera.getAllUnreadResults();
		var rightResults = rightCamera.getAllUnreadResults();

		for (PhotonPipelineResult result : leftResults) {
			leftEstimator.estimateCoprocMultiTagPose(result).ifPresent(otherResult -> swerveEstimator
					.addVisionMeasurement(otherResult.estimatedPose.toPose2d(), otherResult.timestampSeconds));
		}
		for (PhotonPipelineResult result : rightResults) {
			rightEstimator.estimateCoprocMultiTagPose(result).ifPresent(otherResult -> swerveEstimator
					.addVisionMeasurement(otherResult.estimatedPose.toPose2d(), otherResult.timestampSeconds));
		}
		swerveEstimator.update(gyro, swervePositions);
	}

	public PolarPoint getDistanceToHub(Pose2d target) {
		Pose2d myPosition = swerveEstimator.getEstimatedPosition();
		Pose2d relative = myPosition.relativeTo(target);

		return new PolarPoint(Meters.of(Math.hypot(relative.getX(), relative.getY())),
				relative.getRotation().getMeasure());
	}

	public Pose2d getTarget() {
		return Pose2d.kZero;
	}
}
