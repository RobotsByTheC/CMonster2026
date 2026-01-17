package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.IntakeConstants.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

@Logged
public class Intake extends SubsystemBase {
  class Roller extends SubsystemBase {
    public Command stop() {
      return run(() -> io.setIntakeVoltage(Volts.zero()));
    }

    public Command runIntakeMotor() {
      return run(() -> io.setIntakeVoltage(INTAKE_VOLTAGE));
    }

    public Command reverseIntakeMotor() {
      return run(() -> io.setIntakeVoltage(OUTTAKE_VOLTAGE));
    }
  }

  class Extension extends SubsystemBase {
    public Command stop() {
      return run(() -> io.setWristVoltage(Volts.zero()));
    }

    public Command stow() {
      return run(
          () -> io.setWristPosition(WRIST_STOW_ANGLE)
      ).withName("Stow intake");
    }

    public Command extend() {
      return run(
          () -> io.setWristPosition(WRIST_EXTEND_ANGLE)
      ).withName("Extend intake");
    }
  }

  private final IntakeIO io;
  private final Roller roller;
  private final Extension extension;

  private final Trigger atSetpoint;

  public Intake(IntakeIO io) {
    this.io = io;
    roller = new Roller();
    extension = new Extension();

    atSetpoint = new Trigger(io::isWristAtSetpoint).debounce(1);

    roller.setDefaultCommand(roller.stop());
    extension.setDefaultCommand(extension.stop());
  }

  public Command extendAndIntake() {
    return claimCommand(extension.extend().alongWith(roller.runIntakeMotor()));
  }

  public Command retract() {
    return claimCommand(extension.stow().until(atSetpoint)).deadlineFor(roller.runIntakeMotor());
  }

  public Command c_idle() {
    return idle().alongWith(extension.stow());
  }

  // commands v3
//  public Commandv3 idle() {
//    return run(coroutine -> {
//      coroutine.await(extension.stow(), roller.stopIntakeMotor());
//    }).named("Idle");
//  }

  private Command claimCommand(Command command) {
    command.addRequirements(this);
    return command;
  }
}