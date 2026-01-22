package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.IntakeConstants.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Shooter extends SubsystemBase {
	private final ShooterIO io;
	public Shooter(ShooterIO io) {
		this.io = io;
	}

	public Command runAtSpeed(Supplier<AngularVelocity> speedsupplier) {
		return run(() -> io.setSpeed(speedsupplier.get())).withName("ShooterRunAtSpeed");
	public Command o_stop() {
		return runOnce(io::stop);
	}

}
