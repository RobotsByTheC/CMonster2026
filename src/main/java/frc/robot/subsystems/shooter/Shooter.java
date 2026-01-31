package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.data.LookupTable;
import java.util.function.Supplier;

@Logged
public class Shooter extends SubsystemBase {
	private final ShooterIO io;

	private final MutDistance lastDistanceToTarget = Meters.mutable(0);

	public Shooter(ShooterIO io) {
		this.io = io;
	}

	public Command f_shootDistance(Supplier<Distance> distance) {
		return run(() -> {
			if (!lastDistanceToTarget.isNear(distance.get(), Inches.of(0.5)) || lastDistanceToTarget.magnitude() == 0) {
				io.setFlywheelVelocity(LookupTable.SPEED_TABLE.get(distance.get()));
				io.setHoodAngle(LookupTable.ANGLE_TABLE.get(distance.get()));
				lastDistanceToTarget.mut_setMagnitude(distance.get().magnitude());
			}
		});
	}

	public Command o_resetDistance() {
		return runOnce(() -> lastDistanceToTarget.mut_setMagnitude(0));
	}

	public Command f_idle() {
		return run(() -> {
			io.stopFlywheel();
			io.stopHood();
		});
	}
}
