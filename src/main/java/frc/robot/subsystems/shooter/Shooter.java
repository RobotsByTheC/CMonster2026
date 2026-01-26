package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Robot;

@Logged
public class Shooter extends SubsystemBase {
	private final ShooterIO io;
  private Distance lastDistanceToTarget = Meters.zero();

	public Shooter(ShooterIO io) {
		this.io = io;
	}

  public Command f_shootDistance(Supplier<Distance> distance) {
    return run(() -> {
      if (!distance.get().isNear(lastDistanceToTarget, Inches.of(0.01)) || lastDistanceToTarget.isEquivalent(Meters.zero())) {
        io.setFlywheelVelocity(() -> LookupTable.SPEED_TABLE.get(distance.get()));
        io.setHoodAngle(() -> LookupTable.ANGLE_TABLE.get(distance.get()));
        lastDistanceToTarget = distance.get();
      }
    });
  }

	public Command o_stop() {
		return runOnce(io::stopFlywheel).andThen(runOnce(io::stopHood)).andThen(Commands.runOnce(() -> lastDistanceToTarget = Meters.zero()));
	}
}
