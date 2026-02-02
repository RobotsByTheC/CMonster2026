package frc.robot.subsystems.hopper;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.HopperConstants.INTAKE_VOLTAGE;
import static frc.robot.Constants.HopperConstants.OUTTAKE_VOLTAGE;

@Logged
public class Hopper extends SubsystemBase {
	private final HopperIO io;

	public Hopper(HopperIO io) {
		this.io = io;
	}

	public Command f_hopperIntake() {
		return run(() -> {
			io.setVoltage(INTAKE_VOLTAGE);
		});
	}

	public Command f_hopperReverse() {
		return run(() -> {
			io.setVoltage(OUTTAKE_VOLTAGE);
		});
	}

	public Command f_idle() {
		return run(() -> {
			io.stop();
		});
	}
}
