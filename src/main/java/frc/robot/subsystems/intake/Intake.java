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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

@Logged
public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final ArmFeedforward feedforward;
  private final ProfiledPIDController pidController;
  @NotLogged private final SysIdRoutine sysIdRoutine;

  public Intake(IntakeIO io) {
    this.io = io;
    pidController = new ProfiledPIDController(KP, KI, KD, new TrapezoidProfile.Constraints(MAX_WRIST_SPEED.in(RadiansPerSecond), MAX_WRIST_ACCELERATION.in(RadiansPerSecondPerSecond)));
    pidController.setTolerance(WRIST_TOLERANCE.in(Radians));
    pidController.enableContinuousInput(0, 2*Math.PI);
    feedforward = new ArmFeedforward(KS, KG, KV, KA);
    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(Volts.per(Second).of(0.5), Volts.of(4), null),
            new SysIdRoutine.Mechanism(io::setWristVoltage, null, this));
  }

  public Command runSysIdRoutine() {
    return sysIdRoutine
        .dynamic(SysIdRoutine.Direction.kForward)
        .until(() -> io.getWristPosition().gte(Radians.of(Math.PI)))
        .andThen(sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).until(() -> io.getWristPosition().lte(Radians.zero())))
        .andThen(sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).until(() -> io.getWristPosition().gte(Radians.of(Math.PI))))
        .andThen(sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).until(() -> io.getWristPosition().lte(Radians.zero())));
  }

  public Command stow() {
    return startRun(
        () -> pidController.reset(io.getWristPosition().in(Radians), io.getWristVelocity().in(RadiansPerSecond)),
        () -> io.setWristVoltage(calculatePIDVoltage(WRIST_STOW_ANGLE))
    );
  }
  public Command extend() {
    return startRun(
        () -> pidController.reset(io.getWristPosition().in(Radians), io.getWristVelocity().in(RadiansPerSecond)),
        () -> io.setWristVoltage(calculatePIDVoltage(WRIST_EXTEND_ANGLE))
    );
  }

  public Command runIntakeMotor() {
    return Commands.run(() -> io.setIntakeVoltage(INTAKE_VOLTAGE));
  }

  public Command stopIntakeMotor() {
    return Commands.run(() -> io.setIntakeVoltage(Volts.zero()));
  }

  public Command reverseIntakeMotor() {
    return Commands.run(() -> io.setIntakeVoltage(OUTTAKE_VOLTAGE));
  }

  private Voltage calculatePIDVoltage(Angle targetAngle) {
    return Volts.of(pidController.calculate(io.getWristPosition().in(Radians), targetAngle.in(Radians)) + feedforward.calculate(io.getWristPosition().in(Radians), io.getWristVelocity().in(RadiansPerSecond)));
  }
}