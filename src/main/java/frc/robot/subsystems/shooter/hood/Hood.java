package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ShooterConstants.HoodConstants.*;

@Logged
public class Hood extends SubsystemBase {
	private final HoodIO io;
	private final ProfiledPIDController pidController;
	private final ArmFeedforward feedforward;

	public Hood(HoodIO io) {
		this.io = io;
		pidController = new ProfiledPIDController(KP, KI, KD, new TrapezoidProfile.Constraints(
				MAX_VELOCITY.in(RadiansPerSecond), MAX_ACCELERATION.in(RadiansPerSecondPerSecond)));
		feedforward = new ArmFeedforward(KS, KG, KV, KA);
	}

	public boolean isAtTargetAngle() {
		return Radians.of(pidController.getSetpoint().position).isNear(io.getAngle(), Degrees.of(1));
	}

	private Voltage calculatePIDVoltage(Angle targetAngle) {
		return Volts.of(pidController.calculate(io.getAngle().in(Radians), targetAngle.in(Radians))
				+ feedforward.calculate(io.getAngle().in(Radians), io.getVelocity().in(RadiansPerSecond)));
	}

	public Command f_holdDesiredAngle(Supplier<Angle> target) {
		return startRun(() -> pidController.reset(io.getAngle().in(Radians), io.getVelocity().in(RadiansPerSecond)),
				() -> io.setVoltage(calculatePIDVoltage(target.get())));
	}

	public Command l_returnToNormalcy() {
		return run(() -> io.setVoltage(Volts.of(-1))).until(io::atBottom);
	}

	public Command o_stop() {
		return runOnce(io::stop);
	}

	public Command f_idle() {
		return o_stop().andThen(idle());
	}
}
