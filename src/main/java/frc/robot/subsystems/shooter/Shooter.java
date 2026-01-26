package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ShooterConstants.SHOOTER_SPEED;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;

@Logged
public class Shooter extends SubsystemBase {
	private final ShooterIO io;
  public double lastDistanceToTarget = 0;

	public Shooter(ShooterIO io) {
		this.io = io;
	}

  private void updateTarget(Distance distanceToTarget) {
    int newDistance = (int) Math.round(distanceToTarget.in(Meters) * 10);
    if (newDistance == lastDistanceToTarget) return;
    Pair<AngularVelocity, Angle> lookup = LookupTable.SHOOTER_LOOKUP_TABLE.get(newDistance);
    io.setFlywheelVelocity(lookup.getFirst());
    io.setHoodAngle(lookup.getSecond());
    lastDistanceToTarget = newDistance;
  }

  public Command f_shootAtTarget(Supplier<Distance> distance) {
    return run(() -> updateTarget(distance.get()));
  }
}
