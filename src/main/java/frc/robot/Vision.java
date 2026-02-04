package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import org.photonvision.PhotonCamera;

@Logged
public class Vision {
	private final PhotonCamera rightCamera = new PhotonCamera("OV9281-1");
	private final PhotonCamera leftCamera = new PhotonCamera("OV9281-2");
	public Pose2d getTarget() {
		return Pose2d.kZero;
	}
}
