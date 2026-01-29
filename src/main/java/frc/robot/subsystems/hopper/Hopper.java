package frc.robot.subsystems.hopper;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.HopperConstants.IntakeVoltage;
import static frc.robot.Constants.HopperConstants.ReverseVoltage;

@Logged
public class Hopper extends SubsystemBase {
	private final HopperIO io;

	public Hopper(HopperIO io) {
		this.io = io;
	}

	public Command f_hopperIntake() {
		return run(() -> {
			io.setVoltage(IntakeVoltage);
		});
	}

	public Command f_hopperReverse() {
		return run(() -> {
			// Set this to negative velocity
			io.setVoltage(ReverseVoltage);
		});
	}

	public Command f_idle() {
		return run(() -> {
			io.stop();
		});
	}
}
