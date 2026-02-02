package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.IntakeConstants.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

@Logged
public class Intake extends SubsystemBase {
	private final IntakeIO io;
	private final Roller roller;
	private final Extension extension;
	private final ArmFeedforward feedforward;
	private final ProfiledPIDController pidController;
	@NotLogged private final SysIdRoutine sysIdRoutine;

	class Extension extends SubsystemBase {
		public Command extend() {
			return rotateToAngle(WRIST_EXTEND_ANGLE);
		}

		public Command stow() {
			return rotateToAngle(WRIST_STOW_ANGLE);
		}

		public Command stop() {
			return runOnce(() -> io.setWristVoltage(Volts.zero()));
		}

		private Command rotateToAngle(Angle targetAngle) {
			return startRun(
					() -> pidController.reset(io.getWristPosition().in(Radians),
							io.getWristVelocity().in(RadiansPerSecond)),
					() -> io.setWristVoltage(calculatePIDVoltage(targetAngle)));
		}

		private Voltage calculatePIDVoltage(Angle targetAngle) {
			return Volts
					.of(pidController.calculate(io.getWristPosition().in(Radians), targetAngle.in(Radians)) + feedforward
							.calculate(io.getWristPosition().in(Radians), io.getWristVelocity().in(RadiansPerSecond)));
		}
	}

	class Roller extends SubsystemBase {
		public Command runIntakeMotor() {
			return runOnce(() -> io.setIntakeVoltage(INTAKE_VOLTAGE));
		}

		public Command stop() {
			return runOnce(() -> io.setIntakeVoltage(Volts.zero()));
		}

		public Command reverseIntakeMotor() {
			return runOnce(() -> io.setIntakeVoltage(OUTTAKE_VOLTAGE));
		}
	}

	public Intake(IntakeIO io) {
		this.io = io;
		roller = new Roller();
		roller.setDefaultCommand(roller.stop());
		extension = new Extension();
		extension.setDefaultCommand(extension.stop());
		pidController = new ProfiledPIDController(KP, KI, KD, new TrapezoidProfile.Constraints(
				MAX_WRIST_SPEED.in(RadiansPerSecond), MAX_WRIST_ACCELERATION.in(RadiansPerSecondPerSecond)));
		pidController.setTolerance(0.0001);
		pidController.enableContinuousInput(0, 2 * Math.PI);
		feedforward = new ArmFeedforward(KS, KG, KV, KA);
		sysIdRoutine = new SysIdRoutine(new SysIdRoutine.Config(Volts.per(Second).of(0.5), Volts.of(4), null),
				new SysIdRoutine.Mechanism(io::setWristVoltage, null, this));
	}

	public Command runSysIdRoutine() {
		return sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward)
				.until(() -> io.getWristPosition().gte(Radians.of(Math.PI)))
				.andThen(sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse)
						.until(() -> io.getWristPosition().lte(Radians.zero())))
				.andThen(sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)
						.until(() -> io.getWristPosition().gte(Radians.of(Math.PI))))
				.andThen(sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse)
						.until(() -> io.getWristPosition().lte(Radians.zero())));
	}

	public Command claim(Command command) {
		command.addRequirements(this);
		return command;
	}

	// f before a method means forever, l means it has an end condition, o means run once.

	public Command f_stowAndIdle() {
		return claim(extension.stow()).withName("Intake Stow & Idle");
	}

	public Command f_extendAndGrab() {
		return claim(extension.extend().alongWith(roller.runIntakeMotor())).withName("Intake Extend & Grab");
	}

	public Command l_retractAndGrab() {
		return claim(extension.stow().until(pidController::atSetpoint).deadlineFor(roller.runIntakeMotor()))
				.withName("Intake Retract & Grab");
	}
}
