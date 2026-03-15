package frc.robot;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.VisionConstants.BLUE_HUB;
import static frc.robot.Constants.VisionConstants.FRONT_CAMERA_OFFSET;
import static frc.robot.Constants.VisionConstants.REAR_CAMERA_OFFSET;
import static frc.robot.Constants.VisionConstants.RED_HUB;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.data.LookupTable;
import frc.robot.data.PolarPoint;
import java.util.function.BooleanSupplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Pose estimation class. This uses multi-tag pose estimation from PhotonVision (which requires each camera to have 2 or
 * more AprilTags in view to get any data!) and swerve module odometry to estimate the position of the robot on the
 * field. Robot heading is based purely on the gyro measurement, which is much more accurate than both vision and
 * odometry.
 */
@Logged
public class PoseEstimation {
  private final BooleanSupplier DASHBOARD_FIELD_FRONT_CAMERA_CONNECTED;
  private final BooleanSupplier DASHBOARD_FIELD_REAR_CAMERA_CONNECTED;

  // Loading the field JSON is very slow. Storing it in a static final field means we only need to
  // load it once.
  private static final AprilTagFieldLayout fieldTags = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  // 1/2 means results from <2 meters are more trusted than normal, while results from >2 meters
  // are less trusted than normal
  private static final double DISTANCE_SCALE_FACTOR = 0.5;

  /**
   * How far a pose estimate from vision can be from the last estimated position for it to be discarded. Note that
   * because we update at about 50Hz, this is a limit on maximum robot velocity (eg 1 foot in 20ms -> 50 feet per second,
   * well over the physical limit of about 16 ft/s)
   */
  private static final Distance TELEPORTATION_LIMIT = Feet.of(1);

  private static final Vector<N3> BASE_VISION_STDDEVS = VecBuilder.fill(
      // X
      Inches.of(4).in(Meters),
      // Y (should be the same as X)
      Inches.of(4).in(Meters),
      // Heading - the super high std. dev means only the gyro heading get used
      Rotations.of(99999).in(Radians));

  private final PhotonCamera frontCamera;
  private final PhotonCamera rearCamera;

  private final PhotonPoseEstimator frontEstimator;
  private final PhotonPoseEstimator rearEstimator;
  private final SwerveDrivePoseEstimator swerveEstimator;

  public PoseEstimation() {
    frontCamera = new PhotonCamera("OV9281-1");
    rearCamera = new PhotonCamera("OV9281-2");

    DASHBOARD_FIELD_FRONT_CAMERA_CONNECTED = frontCamera::isConnected;
    DASHBOARD_FIELD_REAR_CAMERA_CONNECTED = rearCamera::isConnected;

    frontEstimator = new PhotonPoseEstimator(fieldTags, FRONT_CAMERA_OFFSET);
    rearEstimator = new PhotonPoseEstimator(fieldTags, REAR_CAMERA_OFFSET);

    // Start at (0, 0) with no rotation. This is the blue alliance outpost corner, facing directly
    // at the red alliance wall on the other end of the field.
    // Vision estimates will update the (X, Y) position and rapidly converge to the correct position,
    // but won't update the heading. If we're on the red alliance, we'll either need to turn
    // the robot on facing the same absolute direction as on blue, reset the gyro heading 180°
    // after getting the alliance color, or gather vision data while disabled to approximate the
    // true heading based on AprilTag data.
    swerveEstimator = new SwerveDrivePoseEstimator(Constants.SwerveConstants.DriveConstants.KINEMATICS, Rotation2d.kZero,
        new SwerveModulePosition[]{new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(),
            new SwerveModulePosition()},
        Pose2d.kZero);
  }

  public void update(Rotation2d gyro, SwerveModulePosition[] swervePositions) {
    var frontResults = frontCamera.getAllUnreadResults();
    var rearResults = rearCamera.getAllUnreadResults();

    for (PhotonPipelineResult result : frontResults) {
      // Note: this will only do anything the result has 2 or more tags in view
      frontEstimator.estimateCoprocMultiTagPose(result).ifPresent(this::applyVisionPose);
    }

    for (PhotonPipelineResult result : rearResults) {
      // Note: this will only do anything the result has 2 or more tags in view
      rearEstimator.estimateCoprocMultiTagPose(result).ifPresent(this::applyVisionPose);
    }

    swerveEstimator.update(gyro, swervePositions);
  }

  @Logged private Pose2d lastVisionEstimate = Pose2d.kZero;

  private void applyVisionPose(EstimatedRobotPose visionEstimate) {
    if (visionEstimate.targetsUsed.isEmpty()) {
      // Sanity check: the estimate should use at least one target
      return;
    }

    Pose3d estimatedPose3d = visionEstimate.estimatedPose;
    Pose2d estimatedPose = estimatedPose3d.toPose2d();
    this.lastVisionEstimate = estimatedPose; // for logging

    if (estimatedPose.minus(getPose()).getTranslation().getNorm() > TELEPORTATION_LIMIT.in(Meters)) {
      // The vision estimate places us too far away from the previously estimated position
      // TODO: The early exit is commented about because excessive wheel slip will cause all
      // vision measurements to be rejected. Consider tracking vision-based estimates over
      // time and only reject ones that differ too much from the latest accepted estimates
      // return;
    }

    // The average distance to the detected AprilTags
    double averageDistance = 0;

    for (PhotonTrackedTarget target : visionEstimate.targetsUsed) {
      // Distance to this target, in meters
      double targetDistance = target.getAlternateCameraToTarget().getTranslation().getNorm();
      averageDistance += targetDistance;
    }
    averageDistance /= visionEstimate.targetsUsed.size();

    double stdDevScale = getStdDevScale(visionEstimate, averageDistance);

    swerveEstimator.addVisionMeasurement(estimatedPose, visionEstimate.timestampSeconds,
        BASE_VISION_STDDEVS.times(stdDevScale));
  }

  private static double getStdDevScale(EstimatedRobotPose visionEstimate, double averageDistance) {
    // Standard deviation scale factor. Starts at 1 for no scaling
    double stdDevScale = 1;

    // Divide by the number of targets used. Using more targets means the results are more accurate,
    // so the standard deviation would go down.
    // Note that in practice, only between 2 and ~6 AprilTags will be in view at any one time
    stdDevScale /= visionEstimate.targetsUsed.size();

    // Scale by the average distance to targets. Targets that are farther away are less reliable,
    // so the standard deviation would go up. The scale factor is currently arbitrary.
    stdDevScale *= (averageDistance * DISTANCE_SCALE_FACTOR);

    return stdDevScale;
  }

  public PolarPoint getDistanceToPose(Pose2d target) {
    Pose2d myPosition = swerveEstimator.getEstimatedPosition();
    Pose2d relative = myPosition.relativeTo(target);
    System.out.println("x: " + relative.getX() + ", y: " + relative.getY());

    return new PolarPoint(Meters.of(Math.hypot(relative.getX(), relative.getY())), relative.getRotation().getMeasure());
  }

  public Pose2d getEstimatedPosition() {
    return swerveEstimator.getEstimatedPosition();
  }

  public Distance getDistanceToBlueHub() {
    return getDistanceToPose(BLUE_HUB).distance();
  }

  public Angle getAngleToBlueHub() {
    return getDistanceToPose(BLUE_HUB).angle();
  }

  public Angle getAngleToRedHub() {
    return getDistanceToPose(RED_HUB).angle();
  }

  public Distance getDistanceToRedHub() {
    return getDistanceToPose(RED_HUB).distance();
  }

  public AngularVelocity getTargetVelocity() {
    return LookupTable.getVelocity();
  }

  public Angle getTargetHoodAngle() {
    return LookupTable.getAngle();
  }

  public Pose2d getPose() {
    return swerveEstimator.getEstimatedPosition();
  }
}
