package frc.robot;

import static frc.robot.Constants.VisionConstants.leftOffset;
import static frc.robot.Constants.VisionConstants.rightOffset;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.filter.RepetitiveDebouncer;
import java.util.Comparator;
import java.util.List;
import java.util.Set;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

@Logged
public class Vision {
  public static final int NO_TAG = 0;
  public static final Pose3d NO_TARGET = Pose3d.kZero;

  private PhotonTrackedTarget nearestReefAprilTag;
  private Pose3d nearestReefAprilTagTransform = NO_TARGET;
  private int nearestTagId = NO_TAG;

  private final RepetitiveDebouncer seesTagDebouncer = new RepetitiveDebouncer(8, false);

  @Logged private Transform3d leftTransform = new Transform3d();
  @Logged private Transform3d rightTransform = new Transform3d();

  public final Trigger seesTagTrigger = new Trigger(this::seesTag);

  private final PhotonCamera right = new PhotonCamera("OV9281-1");
  private final PhotonCamera left = new PhotonCamera("OV9281-2");
  private final Set<Integer> reefIDs = Set.of(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22);
  private Pose3d lastRealValue = Pose3d.kZero;
  private Rotation2d lastRealRotation = Rotation2d.kZero;

  public void update() {
    var leftResults = left.getAllUnreadResults();
    var rightResults = right.getAllUnreadResults();

    // Find the ID number of the nearest reef AprilTag

    nearestTagId =
        leftResults.stream()
            .filter(
                lr ->
                    rightResults.stream()
                        .anyMatch(
                            rr -> {
                              if (rr.getBestTarget() != null && lr.getBestTarget() != null) {
                                return rr.getBestTarget().getFiducialId()
                                    == lr.getBestTarget().getFiducialId();
                              } else {
                                return false;
                              }
                            }))
            .min(
                Comparator.comparingDouble(
                    r ->
                        getTransformRelativeToRobot(r.getBestTarget(), leftOffset)
                            .getTranslation()
                            .getNorm()))
            .map(r -> r.getBestTarget().getFiducialId())
            .orElse(NO_TAG);

    // Find the pose of the nearest reef AprilTag, with some debugging logging for the raw
    // transforms from the cameras

    PhotonTrackedTarget bestOverall = null;
    PhotonTrackedTarget bestLeft = getClosestTarget(leftResults, leftOffset);
    PhotonTrackedTarget bestRight = getClosestTarget(rightResults, rightOffset);

    if (bestLeft == null) {
      bestOverall = bestRight;
      if (bestOverall != null) {
        nearestReefAprilTagTransform = getTransformRelativeToRobot(bestOverall, rightOffset);
      } else {
        nearestReefAprilTagTransform = NO_TARGET;
      }
    } else if (bestRight == null) {
      bestOverall = bestLeft;
      nearestReefAprilTagTransform = getTransformRelativeToRobot(bestOverall, leftOffset);
    } else {
      Pose3d leftPose = getTransformRelativeToRobot(bestLeft, leftOffset);
      Pose3d rightPose = getTransformRelativeToRobot(bestRight, rightOffset);
      Rotation3d leftRotation = leftPose.getRotation();
      Rotation3d rightRotation = rightPose.getRotation();

      nearestReefAprilTagTransform =
          new Pose3d(
              (leftPose.getX() + rightPose.getX()) / 2,
              (leftPose.getY() + rightPose.getY()) / 2,
              (leftPose.getZ() + rightPose.getZ()) / 2,
              new Rotation3d(
                  (leftRotation.getX() + rightRotation.getX()) / 2,
                  (leftRotation.getY() + rightRotation.getY()) / 2,
                  (leftRotation.getZ() + rightRotation.getZ()) / 2));
    }
    nearestReefAprilTag = bestOverall;
    if (bestLeft == null) {
      leftTransform = new Transform3d();
    } else {
      leftTransform = bestLeft.bestCameraToTarget;
    }
    if (bestRight == null) {
      rightTransform = new Transform3d();
    } else {
      rightTransform = bestRight.bestCameraToTarget;
    }

    if (nearestReefAprilTagTransform != null
        && !nearestReefAprilTagTransform.equals(Pose3d.kZero)) {
      lastRealValue = nearestReefAprilTagTransform;
      lastRealRotation = lastRealValue.getRotation().toRotation2d().plus(Rotation2d.k180deg);
    }
    seesTagDebouncer.addValue(seesTag());
  }

  /**
   * Gets the ID of the nearest detected AprilTag seen by both cameras. Returns {@link #NO_TAG} if
   * no AprilTag is detected by both cameras.
   */
  @SuppressWarnings("unused")
  public int getNearestTagId() {
    return nearestTagId;
  }

  public boolean seesTag() {
    return nearestTagId != NO_TAG;
  }

  @SuppressWarnings("unused")
  public Pose3d getRobotTransformNearestToReef() {
    return nearestReefAprilTagTransform;
  }

  @SuppressWarnings("unused")
  public PhotonTrackedTarget getNearestReefAprilTag() {
    return nearestReefAprilTag;
  }

  private Pose3d getTransformRelativeToRobot(PhotonTrackedTarget target, Pose3d cameraPosition) {
    return cameraPosition.transformBy(target.bestCameraToTarget);
  }

  private PhotonTrackedTarget getClosestTarget(
      List<PhotonPipelineResult> pipelineResults, Pose3d cameraPosition) {
    return pipelineResults.stream()
        .flatMap(result -> result.getTargets().stream())
        .filter(target -> reefIDs.contains(target.getFiducialId()))
        .min(
            Comparator.comparingDouble(
                target ->
                    getTransformRelativeToRobot(target, cameraPosition).getTranslation().getNorm()))
        .orElse(null);
  }

  @SuppressWarnings("unused")
  public Pose3d getLastRealValue() {
    return lastRealValue;
  }

  public boolean canSeeFilteredTag() {
    return !seesTagDebouncer.getBoolean();
  }
}
