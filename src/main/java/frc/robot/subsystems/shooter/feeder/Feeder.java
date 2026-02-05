package frc.robot.subsystems.shooter.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.FeederConstants.*;

public class Feeder extends SubsystemBase {
	private final FeederIO io;

	public Feeder(FeederIO io) {
		this.io = io;
	}

	public Command f_feederIntake() {
		return run(() -> {
			io.setVoltage(FEED_VOLTAGE);
		});
	}

	public Command f_feederReverse() {
		return run(() -> {
			io.setVoltage(SPIT_VOLTAGE);
		});
	}

	public Command f_idle() {
		return run(io::stop);
	}
}
