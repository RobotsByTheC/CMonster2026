package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;

@Logged
public class Shooter extends SubsystemBase {
	private final ShooterIO io;

	public Shooter(ShooterIO io) {
		this.io = io;
	}

	public Command o_stopFlywheel() {
		return runOnce(io::stopFlywheel);
	}

	public Command f_shoot(Supplier<AngularVelocity> desiredSpeed) {
		return run(() -> io.setFlywheelVelocity(desiredSpeed.get()))
				.withName("Shooter @ " + desiredSpeed.get().in(RPM) + " RPM");
	}
}
