package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

@Logged
public class Flywheel extends SubsystemBase {
	private final FlywheelIO io;

	public Flywheel(FlywheelIO io) {
		this.io = io;
	}

  @NotLogged
  public boolean atTargetSpeed() {
    return io.atTargetVelocity();
  }

	public Command o_stop() {
		return runOnce(io::stop);
	}

	public Command f_shoot(Supplier<AngularVelocity> target) {
		return run(() -> io.setVelocity(target.get()));
	}

	public Command f_idle() {
		return o_stop().andThen(idle());
	}
}
