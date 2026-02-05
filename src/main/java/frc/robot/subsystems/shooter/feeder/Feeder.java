package frc.robot.subsystems.shooter.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.FeederConstants.*;

public class Feeder extends SubsystemBase {
	private final FeederIO io;
	private final Trigger readyToFire;

	public Feeder(FeederIO io, Trigger readyToFire) {
		this.io = io;

		this.readyToFire = readyToFire.and(io::isBallReadyToFire);
	}

	public boolean canShoot() {
		return io.isBallReadyToFire();
	}

	public Command queueBall() {
		return idle().until(readyToFire).andThen(activate().until(io::isBallAtFlywheel)).andThen(stop());
	}

	public Command activate() {
		return run(() -> io.setVoltage(FEED_VOLTAGE));
	}

	public Command stop() {
		return run(io::stop);
	}
}
