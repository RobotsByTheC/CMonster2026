package frc.robot;

import static frc.robot.Constants.VisionConstants.leftOffset;
import static frc.robot.Constants.VisionConstants.rightOffset;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import java.util.Comparator;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

@Logged
public class Vision {
  public static final int NO_TAG = 0;
  public static final Pose3d NO_TARGET = Pose3d.kZero;

  private final PhotonCamera rightCamera = new PhotonCamera("OV9281-1");
  private final PhotonCamera leftCamera = new PhotonCamera("OV9281-2");

  private Pose3d lastRealLeftPose = NO_TARGET;
  private Pose3d lastRealRightPose = NO_TARGET;
  private Pose3d lastRealPose = NO_TARGET;

  public void update() {
    var leftResults = leftCamera.getAllUnreadResults();
    var rightResults = rightCamera.getAllUnreadResults();

    Pose3d leftPose =
        getTransformRelativeToRobot(getClosestTarget(leftResults, leftOffset), leftOffset);
    Pose3d rightPose =
        getTransformRelativeToRobot(getClosestTarget(rightResults, rightOffset), rightOffset);

    if (leftPose != null && !leftPose.equals(NO_TARGET)) lastRealLeftPose = leftPose;
    if (rightPose != null && !rightPose.equals(NO_TARGET)) lastRealRightPose = rightPose;

    lastRealPose =
        new Pose3d(
            (lastRealLeftPose.getX() + lastRealRightPose.getX()) / 2,
            (lastRealLeftPose.getY() + lastRealRightPose.getY()) / 2,
            (lastRealLeftPose.getZ() + lastRealRightPose.getZ()) / 2,
            new Rotation3d(
                (lastRealLeftPose.getRotation().getX() + lastRealRightPose.getRotation().getX())
                    / 2,
                (lastRealLeftPose.getRotation().getY() + lastRealRightPose.getRotation().getY())
                    / 2,
                (lastRealLeftPose.getRotation().getZ() + lastRealRightPose.getRotation().getZ())
                    / 2));
  }

  private Pose3d getTransformRelativeToRobot(PhotonTrackedTarget target, Pose3d cameraPosition) {
    return cameraPosition.transformBy(target.bestCameraToTarget);
  }

  private PhotonTrackedTarget getClosestTarget(
      List<PhotonPipelineResult> pipelineResults, Pose3d cameraPosition) {
    return pipelineResults.stream()
        .flatMap(result -> result.getTargets().stream())
        .min(
            Comparator.comparingDouble(
                target ->
                    getTransformRelativeToRobot(target, cameraPosition).getTranslation().getNorm()))
        .orElse(null);
  }

  public Pose3d getTargetPose() {
    return lastRealPose;
  }

  public Pose3d getLastRealLeftPose() {
    return lastRealLeftPose;
  }

  public Pose3d getLastRealRightPose() {
    return lastRealRightPose;
  }
}
