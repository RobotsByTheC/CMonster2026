package frc.robot;

import static frc.robot.Constants.VisionConstants.leftOffset;
import static frc.robot.Constants.VisionConstants.rightOffset;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.filter.RepetitiveDebouncer;
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

  private Pose3d currentLeftPose;
  private Pose3d currentRightPose;
  private Pose3d lastRealLeftPose = NO_TARGET;
  private Pose3d lastRealRightPose = NO_TARGET;
  private Pose3d lastRealPose = NO_TARGET;

  private final RepetitiveDebouncer DEBOUNCER = new RepetitiveDebouncer(10, false);
  public final Trigger SEES_TAG = new Trigger(DEBOUNCER::getBoolean);

  public void update() {
    var leftResults = leftCamera.getAllUnreadResults();
    var rightResults = rightCamera.getAllUnreadResults();

    currentLeftPose =
        getTransformRelativeToRobot(getClosestTarget(leftResults, leftOffset), leftOffset);
    currentRightPose =
        getTransformRelativeToRobot(getClosestTarget(rightResults, rightOffset), rightOffset);

    createRealValues();

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
    if (target == null) return Pose3d.kZero;

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

  private void createRealValues() {
    boolean seen = false;
    if (currentLeftPose != null && !currentLeftPose.equals(NO_TARGET)) {
      lastRealLeftPose = currentLeftPose;
      DEBOUNCER.addValue(true);
      seen = true;
    }
    if (currentRightPose != null && !currentRightPose.equals(NO_TARGET)) {
      lastRealRightPose = currentRightPose;
      DEBOUNCER.addValue(true);
      seen = true;
    }
    if (!seen) DEBOUNCER.addValue(false);
  }
}
