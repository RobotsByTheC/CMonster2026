package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ConstantTuner;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

@Logged
public class Flywheel extends SubsystemBase {
	private final FlywheelIO io;

	public Flywheel(FlywheelIO io) {
		this.io = io;
	}

	public Command o_stop() {
		return runOnce(() -> io.setVoltage(Volts.zero()));
	}

	public Command f_shoot(Supplier<AngularVelocity> target) {
		return run(() -> io.setVelocity(target.get()));
	}

	public Command f_idle() {
		return o_stop().andThen(idle());
	}

	public Command tune() {
		return ConstantTuner.createRoutine(io::setVoltage, this, () -> io.getVelocity().gte(RPM.of(500)),
				() -> io.getVelocity().lte(RPM.zero()));
	}
}
